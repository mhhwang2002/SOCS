"""Bounded lookahead for warehouse Push decisions."""

from dataclasses import replace
from fractions import Fraction
from math import dist, inf, isfinite
from time import perf_counter

from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginBase


def parse_horizon(value):
    if value == 'fleet':
        return None
    try:
        fraction = Fraction(value)
    except (ValueError, ZeroDivisionError) as error:
        raise ValueError('Expected a fraction or fleet.') from error
    if not 0 < fraction <= 1:
        raise ValueError('Lookahead horizon must be in (0, 1], or fleet.')
    return float(fraction)


class WarehouseLookaheadEngine(TESEnginBase):
    def __init__(self, model, *, horizon, timeout=1., **options):
        super().__init__(model, **options)
        model.controller.add_input_ports('?warehouse_push_choice')
        model.controller.traffic_manager.lookahead_enabled = True
        self.horizon = horizon
        self.timeout = timeout
        self.lookahead_calls = 0
        self.lookahead_timeouts = 0
        self.lookahead_seconds = 0.
        self.deferred_pushes = 0

    def _remaining(self, name):
        points = self.model.controller.roadmap.remaining_subpath(name).coordinates()
        return sum(dist(a, b) for a, b in zip(points, points[1:]))

    def _trial(self, robot, leg, threshold, deadline):
        controller = self.model.controller
        while isfinite(self.model.tNext):
            if perf_counter() >= deadline:
                return None
            if threshold is not None and (controller.roadmap.paths[robot] != leg
                                          or self._remaining(robot) <= threshold):
                delays = (self._remaining(r.name) / r.motion_limits.speed for r in self.model.robots)
                return self.tNow + max(delays)
            super().execute_one_step()
        if controller.completed_orders == controller.received_orders and all(
                state == 'home' for state in controller.robot_status.values()):
            return self.tNow
        return inf

    def execute_one_step(self):
        result = super().execute_one_step()
        controller = self.model.controller
        manager = controller.traffic_manager
        if manager.pending_push is None:
            return result
        started = perf_counter()
        robot, _ = manager.pending_push
        leg = controller.roadmap.paths[robot]
        threshold = None if self.horizon is None else self._remaining(robot) * (1 - self.horizon)
        saved = self.capture_snapshot()
        interval = self.TimeIntervalAnimation
        probe = replace(saved, t_next_animation=saved.t_now + .1) if self.model.collision_check else saved
        costs = []
        try:
            if self.model.collision_check:
                self.TimeIntervalAnimation = .1
            for issue in (True, False):
                self.restore_snapshot(probe)
                manager.lookahead_enabled = False
                self.execute_devs_x(self.tNow, controller, PortValue('?warehouse_push_choice', issue))
                costs.append(self._trial(robot, leg, threshold, started + self.timeout))
                if costs[-1] is None:
                    break
        finally:
            self.TimeIntervalAnimation = interval
            self.restore_snapshot(saved)
        complete = len(costs) == 2 and None not in costs
        issue = not complete or costs[0] <= costs[1]
        self.execute_devs_x(self.tNow, controller, PortValue('?warehouse_push_choice', issue))
        self.lookahead_calls += 1
        self.lookahead_timeouts += not complete
        self.deferred_pushes += not issue
        self.lookahead_seconds += perf_counter() - started
        return result
