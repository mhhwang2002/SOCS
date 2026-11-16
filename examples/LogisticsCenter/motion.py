"""Motion plans for the crane and the towing vehicles."""

from bisect import bisect_left
from functools import lru_cache
from math import atan2, ceil, degrees, dist, floor

from examples.ASRS.asrs_devs import (
    Crane, FORK_APPROACH_Y, FORK_RETRACT_Y, motion_position, motion_profile,
)


def axis_profile(a, b, axis):
    return motion_profile(a, b, getattr(Crane, axis + "_acceleration"),
                          getattr(Crane, axis + "_deceleration"),
                          getattr(Crane, axis + "_max_speed"))


def crane_plan(points, action, count):
    stages = []
    for index, (a, b) in enumerate(zip(points, points[1:]), 1):
        profiles = [axis_profile(a[0], b[0], "x"), axis_profile(a[1], b[1], "z")]
        travel = max((p["total_time"] for p in profiles if p), default=0)
        at_rack = (index <= count) == (action == "retrieve")
        reach = FORK_APPROACH_Y if at_rack else -1.5
        fork = axis_profile(FORK_RETRACT_Y, reach, "fork") if index < len(points) - 1 else None
        handling = 2 * fork["total_time"] + 0.7 if fork else 0
        stages.append((a, b, profiles, travel, handling, fork))
    return stages, sum(travel + handling for _, _, _, travel, handling, _ in stages)


def crane_pose(job, now):
    elapsed = max(0, now - job["start"])
    for index, (a, b, profiles, travel, handling, fork) in enumerate(job["motion"], 1):
        if elapsed < travel:
            point = tuple(motion_position(p, elapsed) if p else x for x, p in zip(a, profiles))
            return point, index - 1, FORK_RETRACT_Y, 0, None
        elapsed -= travel
        if elapsed < handling:
            reach_time = fork["total_time"]
            phase = min(elapsed, handling - elapsed, reach_time)
            lift = 0.12 * max(0, min(1, (elapsed - reach_time) / 0.35))
            reached = index if elapsed >= reach_time + 0.35 else index - 1
            return b, reached, motion_position(fork, phase), lift, (index - 1) % len(job["bins"])
        elapsed -= handling
    return job["points"][-1], len(job["points"]) - 1, FORK_RETRACT_Y, 0, None


def rounded(points, radius=0.85):
    path = [points[0]]
    for a, b, c in zip(points, points[1:], points[2:]):
        incoming, outgoing = dist(a, b), dist(b, c)
        if not incoming or not outgoing:
            continue
        inset = min(radius, incoming * 0.4, outgoing * 0.4)
        start = tuple(y + (x - y) * inset / incoming for x, y in zip(a, b))
        end = tuple(y + (z - y) * inset / outgoing for y, z in zip(b, c))
        for step in range(13):
            t = step / 12
            path.append(tuple((1 - t)**2 * x + 2 * t * (1 - t) * y + t*t*z
                              for x, y, z in zip(start, b, end)))
    path.append(points[-1])
    return path


def agv_plan(legs, speed, handoff_indices):
    path, stages = [legs[0][0]], []
    distance = 0
    for index, leg in enumerate(legs, 1):
        section = rounded(leg)
        length = sum(dist(a, b) for a, b in zip(section, section[1:]))
        if length <= 1e-12:
            continue
        profile = motion_profile(distance, distance + length, 0.7, 0.8, speed)
        dwell = 2.0 if index in handoff_indices else 0
        stages.append((index, profile, dwell))
        distance += length
        path.extend(section[1:])
    duration = sum(profile["total_time"] + dwell for _, profile, dwell in stages)
    return path, stages, duration


def agv_pose(job, now, offset=0):
    elapsed = max(0, now - job["depart"])
    distance, reached = 0, 0
    for index, profile, dwell in job["motion"]:
        travel = profile["total_time"]
        if elapsed < travel + dwell:
            distance = motion_position(profile, min(elapsed, travel))
            if elapsed >= travel + dwell / 2:
                reached = index
            break
        elapsed -= travel + dwell
        distance, reached = profile["target"], index
    ends, segments = path_segments(tuple(job["points"]))
    remaining = min(distance - offset, ends[-1])
    index = bisect_left(ends, remaining)
    a, dx, dy, heading = segments[index]
    local = remaining - (ends[index - 1] if index else 0)
    return (a[0] + dx * local, a[1] + dy * local), heading, reached


@lru_cache(maxsize=128)
def path_segments(points):
    ends, segments = [], []
    length_so_far = 0
    for a, b in zip(points, points[1:]):
        length = dist(a, b)
        if length:
            dx, dy = ((v - u) / length for u, v in zip(a, b))
            length_so_far += length
            ends.append(length_so_far)
            segments.append((a, dx, dy, degrees(atan2(-dx, dy))))
    return ends, segments


def handoff_progress(job, now, stop):
    """Return side-transfer progress during the selected vehicle dwell."""
    elapsed = now - job["depart"]
    for index, profile, dwell in job["motion"]:
        elapsed -= profile["total_time"]
        if index == stop:
            return max(0, min(1, elapsed / dwell))
        elapsed -= dwell
    return 0


TRAFFIC_STEP = 0.25
TRAFFIC_BLOCK = 0.75
TRAFFIC_MARGIN = 0.80


def train_blocks(job, offsets):
    """Reserve short grid blocks touched by the train at each traffic tick."""
    start = job["depart"]
    end = job["start"] + job["duration"]
    for step in range(ceil((end - start) / TRAFFIC_STEP) + 1):
        now = min(end, start + step * TRAFFIC_STEP)
        blocks = set()
        for offset in offsets:
            (x, y), _, _ = agv_pose(job, now, offset)
            for bx in range(floor((x - TRAFFIC_MARGIN) / TRAFFIC_BLOCK),
                            floor((x + TRAFFIC_MARGIN) / TRAFFIC_BLOCK) + 1):
                for by in range(floor((y - TRAFFIC_MARGIN) / TRAFFIC_BLOCK),
                                floor((y + TRAFFIC_MARGIN) / TRAFFIC_BLOCK) + 1):
                    blocks.add((bx, by))
        yield blocks


def traffic_departure(now, candidate, booked_jobs, offsets):
    """Find the first departure that can reserve all of its blocks."""
    first = ceil(now / TRAFFIC_STEP)
    booked = {}
    for job in booked_jobs:
        tick = round(job["depart"] / TRAFFIC_STEP)
        for step, blocks in enumerate(train_blocks(job, offsets)):
            if tick + step >= first:
                booked.setdefault(tick + step, set()).update(blocks)
    if not booked:
        return first * TRAFFIC_STEP
    proposed = list(train_blocks(candidate, offsets))
    for departure in range(first, max(booked) + 1):
        for step, blocks in enumerate(proposed):
            occupied = booked.get(departure + step)
            if occupied is not None and not occupied.isdisjoint(blocks):
                break
        else:
            return departure * TRAFFIC_STEP
    return (max(booked) + 1) * TRAFFIC_STEP
