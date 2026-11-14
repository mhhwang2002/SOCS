"""DEVS-calibrated movement and fork-cycle timing for the two-fork crane."""

from __future__ import annotations

import json
from pathlib import Path

from socs.devs.DEVS import DEVS
from socs.devs.TESEngin import ExecutionResult, TESEnginBase

from examples.ASRS.asrs_devs import (
    CELL_WIDTH,
    HOME_Z,
    HOME_X,
    LEVEL_PITCH,
    LOAD_CENTER_OFFSET,
    LOAD_HEIGHT,
    MAX_FORK_REACH,
    RACK_COLUMNS,
    RACK_LEVELS,
    UNLOAD_REACH,
    UNLOAD_STATION_XS,
)
from .asrs_devs import (
    CRANE_X_ACCELERATION,
    CRANE_X_DECELERATION,
    CRANE_X_MAX_SPEED,
    CRANE_Z_ACCELERATION,
    CRANE_Z_DECELERATION,
    CRANE_Z_MAX_SPEED,
    EVENT_EPSILON,
    FORK_ACCELERATION,
    FORK_DECELERATION,
    FORK_LIFT_HEIGHT,
    FORK_MAX_SPEED,
    FORK_SPACING,
    TwoForkCrane,
)

PROFILE_FORMAT = "socs-asrs-twofork-timing"
PROFILE_VERSION = 2
DEFAULT_TIMING_PROFILE = Path(__file__).with_name("data") / "timing_profile.json"
STATION_NAMES = ("L", "R")


def pose_key(position):
    """Return the stable key for a crane rail/hoist position."""
    return f"{position[0]:.3f},{position[1]:.3f}"


def _axis_time(axis, start, target):
    crane = TwoForkCrane()
    crane._set_axis(axis, start)
    profile = crane._make_profile(axis, target)
    return 0.0 if profile is None else profile["total_time"]


def _move_time(start, target):
    """Use the crane's X/Z motion profiles and simultaneous-axis timing."""
    crane = TwoForkCrane()
    crane.x, crane.z = start
    profiles = (
        crane._make_profile("X", target[0]),
        crane._make_profile("Z", target[1]),
    )
    return max(
        (profile["total_time"] for profile in profiles if profile is not None),
        default=0.0,
    )


def _kinematic_parameters():
    return {
        "rack_columns": RACK_COLUMNS,
        "rack_levels": RACK_LEVELS,
        "cell_width": CELL_WIDTH,
        "level_pitch": LEVEL_PITCH,
        "x_acceleration": CRANE_X_ACCELERATION,
        "x_deceleration": CRANE_X_DECELERATION,
        "x_max_speed": CRANE_X_MAX_SPEED,
        "z_acceleration": CRANE_Z_ACCELERATION,
        "z_deceleration": CRANE_Z_DECELERATION,
        "z_max_speed": CRANE_Z_MAX_SPEED,
        "fork_acceleration": FORK_ACCELERATION,
        "fork_deceleration": FORK_DECELERATION,
        "fork_max_speed": FORK_MAX_SPEED,
        "fork_lift_height": FORK_LIFT_HEIGHT,
        "fork_spacing": FORK_SPACING,
        "max_fork_reach": MAX_FORK_REACH,
        "unload_reach": UNLOAD_REACH,
        "load_height": LOAD_HEIGHT,
        "load_center_offset": LOAD_CENTER_OFFSET,
        "event_epsilon": EVENT_EPSILON,
        "home_position": [HOME_X, HOME_Z],
        "station_positions": [
            [UNLOAD_STATION_XS[0], HOME_Z],
            [UNLOAD_STATION_XS[1], HOME_Z],
        ],
    }


class _TimingCalibrationProbe(DEVS):
    """Advance through measured kinematic intervals using the SOCS engine."""

    def __init__(self, samples):
        super().__init__("TwoForkTimingCalibrationProbe")
        self.samples = tuple(samples)
        self.set_initial_state()

    def set_initial_state(self):
        self.index = 0
        self.started_at = 0.0
        self.measurements = {}

    def tau(self):
        return self.samples[self.index][1] if self.index < len(self.samples) else float("inf")

    def delta_x(self, event):
        return False

    def delta_y(self):
        key, _ = self.samples[self.index]
        now = self.tNext
        self.measurements[key] = now - self.started_at
        self.started_at = now
        self.index += 1

    def get_state(self):
        return self.index, self.started_at, tuple(self.measurements.items())

    def set_state(self, state):
        self.index, self.started_at, measurements = state
        self.measurements = dict(measurements)


def _simulate_samples(samples):
    probe = _TimingCalibrationProbe(samples)
    engine = TESEnginBase(
        probe,
        TimeIntervalContinuous=float("inf"),
        TimeIntervalAnimation=float("inf"),
        TimeSimulationEnd=1e9,
    )
    engine.bVerbose = False
    while probe.index < len(probe.samples):
        result = engine.execute_one_step()
        if result is ExecutionResult.TimeBoundReached:
            raise RuntimeError("Timing calibration exceeded its simulation time bound.")
    return probe.measurements


def calibrate_timing_profile():
    """Simulate station, cell-to-cell, and fork service timing intervals."""
    poses = {}
    travel_from = {name: {} for name in STATION_NAMES}
    travel_to = {}
    travel_between = {}
    cycle_targets = {}
    samples = []
    cell_reach_time = _axis_time("Fork1Reach", 0.0, MAX_FORK_REACH)
    unload_reach_time = _axis_time("Fork1Reach", 0.0, UNLOAD_REACH)
    lift_up_time = _axis_time("Fork1Lift", 0.0, FORK_LIFT_HEIGHT)
    reach_withdraw_time = _axis_time("Fork1Reach", MAX_FORK_REACH, 0.0)
    fork_down_time = max(
        _axis_time("Fork1Reach", UNLOAD_REACH, 0.0),
        _axis_time("Fork1Lift", FORK_LIFT_HEIGHT, 0.0),
    )

    for level in range(1, RACK_LEVELS + 1):
        cell_z = (
            (level - 1) * LEVEL_PITCH
            + LOAD_CENTER_OFFSET
            - LOAD_HEIGHT / 2.0
            + 0.08
        )
        for rail_index in range(RACK_COLUMNS + 1):
            position = (rail_index * CELL_WIDTH, cell_z)
            key = pose_key(position)
            poses[key] = {"rail_x": position[0], "z": position[1], "level": level}
            for station_index, station_name in enumerate(STATION_NAMES):
                station = (UNLOAD_STATION_XS[station_index], HOME_Z)
                key_out = f"travel_from:{station_name}:{key}"
                samples.append((key_out, _move_time(station, position)))
                travel_from[station_name][key] = key_out

                key_back = f"travel_to:{key}:{station_name}"
                samples.append((key_back, _move_time(position, station)))
                travel_to.setdefault(key, {})[station_name] = key_back

                cycle = (
                    lift_up_time
                    + reach_withdraw_time
                    + _move_time(position, station)
                    + unload_reach_time
                    + EVENT_EPSILON
                    + fork_down_time
                )
                cycle_key = f"cycle:{key}:{station_name}"
                samples.append((cycle_key, cycle))
                cycle_targets.setdefault(key, {})[station_name] = cycle_key

    pose_keys = tuple(poses)
    for source_key in pose_keys:
        source = (poses[source_key]["rail_x"], poses[source_key]["z"])
        for target_key in pose_keys:
            target = (poses[target_key]["rail_x"], poses[target_key]["z"])
            sample_key = f"travel_between:{source_key}:{target_key}"
            samples.append((sample_key, _move_time(source, target)))
            travel_between.setdefault(source_key, {})[target_key] = sample_key

    measurements = _simulate_samples(samples)
    profile = {
        "format": PROFILE_FORMAT,
        "version": PROFILE_VERSION,
        "parameters": _kinematic_parameters(),
        "stations": {
            name: {"x": UNLOAD_STATION_XS[index], "z": HOME_Z}
            for index, name in enumerate(STATION_NAMES)
        },
        "poses": poses,
        "travel_from_station": {
            station: {pose: measurements[sample] for pose, sample in values.items()}
            for station, values in travel_from.items()
        },
        "travel_to_station": {
            pose: {station: measurements[sample] for station, sample in values.items()}
            for pose, values in travel_to.items()
        },
        "travel_between_pickups": {
            source: {target: measurements[sample] for target, sample in values.items()}
            for source, values in travel_between.items()
        },
        "fork_approach_time": cell_reach_time + EVENT_EPSILON,
        "fork_lift_withdraw_time": lift_up_time + reach_withdraw_time,
        "unload_service_time": unload_reach_time + EVENT_EPSILON + fork_down_time,
        "fork_lift_to_down_cycle": {
            pose: {station: measurements[sample] for station, sample in values.items()}
            for pose, values in cycle_targets.items()
        },
        "sample_count": len(samples),
        "simulation_events": len(measurements),
        "timing_basis": "SOCS DEVS events using TwoForkCrane acceleration profiles",
    }
    return profile


class TimingProfile:
    """Look up pre-simulated leg, approach, and fork-cycle durations."""

    def __init__(self, data):
        if data.get("format") != PROFILE_FORMAT or data.get("version") != PROFILE_VERSION:
            raise ValueError("Unsupported two-fork timing profile format or version.")
        if data.get("parameters") != _kinematic_parameters():
            raise ValueError("Timing profile does not match the current crane kinematics; regenerate it.")
        self.data = data
        self.approach_time = float(data["fork_approach_time"])

    @classmethod
    def load(cls, path=DEFAULT_TIMING_PROFILE):
        path = Path(path)
        return cls(json.loads(path.read_text(encoding="utf-8")))

    def station_to_pickup(self, station_index, pickup):
        station = STATION_NAMES[station_index]
        return self.data["travel_from_station"][station][pose_key(pickup)]

    def pickup_to_station(self, pickup, station_index):
        station = STATION_NAMES[station_index]
        return self.data["travel_to_station"][pose_key(pickup)][station]

    def between_pickups(self, start, target):
        return self.data["travel_between_pickups"][pose_key(start)][pose_key(target)]

    def fork_lift_to_down(self, pickup, station_index):
        station = STATION_NAMES[station_index]
        return self.data["fork_lift_to_down_cycle"][pose_key(pickup)][station]


def save_timing_profile(path=DEFAULT_TIMING_PROFILE):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    profile = calibrate_timing_profile()
    path.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
    return path, profile
