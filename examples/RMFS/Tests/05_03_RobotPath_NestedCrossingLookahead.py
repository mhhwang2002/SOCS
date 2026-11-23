"""Two coupled crossing decisions in an event-level DEVS traffic model."""

from argparse import ArgumentParser
from heapq import heappop, heappush
from math import inf
from time import perf_counter

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginBase


class CrossingNetwork(DEVSph):
    """Front buffer, rear queue, and downstream crossing shared by R1 and R2."""

    downstream_service = {'R1': (4., 1.), 'R2': (2., 8.)}

    def __init__(self):
        super().__init__('CrossingNetwork', variables=dict(
            events=[], waiting=[], finished={}, decisions=[], pending=(),
            downstream_busy=True, rear_clear=None, exit_clear=False,
            downstream_owner='R3', front_owner=None, tail=[],
            r1_at_front=False))
        self.add_input_ports('?choose')
        self.add_phase('Push', 0., initial=True)
        self.add_phase('Waiting', inf)
        self.add_phase('Running', lambda m: m.delay())
        self.add_internal_transition('Push', 'Waiting',
                                     action=lambda m, e: setattr(m._variable, 'pending', ('inside', 'outside')))
        self.add_external_transition('Waiting', 'Running', input_port='?choose', reschedule=True,
                                     action=lambda m, e: m.choose(e.value))
        self.add_internal_transition('Running', 'Waiting', guard=lambda m, e: m.contention_due(),
                                     action=lambda m, e: m.process())
        self.add_internal_transition('Running', 'Running', guard=lambda m, e: not m.contention_due(),
                                     action=lambda m, e: m.process())

    def delay(self):
        events = self._variable.events
        return events[0][0] - self.tNow if events else inf

    def schedule(self, delay, event, robot=''):
        heappush(self._variable.events, (self.tNow + delay, event, robot))

    def choose(self, choice):
        v = self._variable
        if choice not in v.pending:
            raise ValueError(f'Invalid traffic decision {choice}')
        v.decisions.append((self.tNow, choice))
        v.pending = ()
        if choice in self.downstream_service:
            self.dispatch(choice)
            return
        # R3 initially owns the downstream crossing. A delayed exit blocker
        # clears the front crossing's straight exit at t=6.
        self.schedule(10., 'downstream_release', 'R3')
        self.schedule(6., 'exit_release', 'blocker')
        if choice == 'inside':
            self.schedule(5., 'front_arrival', 'R1')
        else:
            # R4 gets the front crossing before R1. Once R4 clears it, R1
            # leaves the rear queue and enters the front crossing.
            v.front_owner = 'R4'
            self.schedule(9., 'front_release', 'R4')

    def contention_due(self):
        v = self._variable
        return bool(v.events and v.events[0][1] == 'downstream_release' and len(v.waiting) > 1)

    def dispatch(self, robot):
        v = self._variable
        v.waiting.remove(robot)
        v.downstream_busy = True
        v.downstream_owner = robot
        occupancy, tail = self.downstream_service[robot]
        self.schedule(occupancy, 'downstream_release', robot)
        self.schedule(occupancy + tail, 'finish', robot)

    def clear_rear(self):
        self._variable.rear_clear = self.tNow
        self.schedule(4.5, 'downstream_arrival', 'R2')

    def process(self):
        v = self._variable
        _, event, robot = heappop(v.events)
        if event == 'front_arrival':
            if v.rear_clear is None:
                self.clear_rear()
            v.r1_at_front = True
            v.front_owner = None if v.exit_clear else 'R1'
            if v.exit_clear:
                self.schedule(3., 'downstream_arrival', 'R1')
        elif event == 'exit_release':
            v.exit_clear = True
            v.front_owner = 'R4' if v.r1_at_front else v.front_owner
            v.finished[robot] = self.tNow
            if v.r1_at_front:
                self.schedule(3., 'downstream_arrival', 'R1')
                self.schedule(5., 'finish', 'R4')
        elif event == 'front_release':
            v.front_owner = None
            v.finished[robot] = self.tNow
            self.clear_rear()
            self.schedule(3., 'front_arrival', 'R1')
        elif event == 'downstream_arrival':
            v.waiting.append(robot)
            if not v.downstream_busy:
                self.dispatch(robot)
        elif event == 'downstream_release':
            if robot == 'R3':
                v.finished[robot] = self.tNow
            v.downstream_busy = False
            v.downstream_owner = None
            if robot in self.downstream_service:
                v.tail.append(robot)
            if len(v.waiting) > 1:
                v.pending = tuple(v.waiting)
            elif v.waiting:
                self.dispatch(v.waiting[0])
        elif event == 'finish':
            if robot == 'R4':
                v.front_owner = None
            v.finished[robot] = self.tNow


def run(depth, *, record=False):
    model = CrossingNetwork()
    engine = TESEnginBase(model)
    engine.bVerbose = False
    candidates = []
    leaves = []
    started = perf_counter()
    frames = []

    def capture():
        v = model._variable
        frames.append(dict(time=engine.tNow, waiting=tuple(v.waiting), finished=tuple(v.finished),
                           downstream=v.downstream_owner, front=v.front_owner, tail=tuple(v.tail),
                           rear_clear=v.rear_clear, r1_at_front=v.r1_at_front,
                           decisions=tuple(v.decisions)))

    def apply(choice):
        engine.execute_devs_x(engine.tNow, model, PortValue('?choose', choice))

    def simulate(remaining, branch=()):
        while model._variable.pending or model.tNext < inf:
            choices = model._variable.pending
            if not choices:
                engine.execute_one_step()
                if record and not branch:
                    capture()
                continue
            selected = choices[0]
            if remaining:
                snapshot = engine.capture_snapshot()
                scores = {}
                try:
                    for choice in choices:
                        engine.restore_snapshot(snapshot)
                        apply(choice)
                        scores[choice] = simulate(remaining - 1, (*branch, choice))
                finally:
                    engine.restore_snapshot(snapshot)
                selected = min(scores, key=scores.get)
                candidates.append((snapshot.t_now, branch, scores))
            apply(selected)
            if record and not branch:
                capture()
        finished = model._variable.finished
        if set(finished) != {'R1', 'R2', 'R3', 'R4', 'blocker'}:
            raise RuntimeError('A rollout did not finish every robot.')
        result = max(finished.values())
        if branch:
            leaves.append((tuple(choice for _, choice in model._variable.decisions), result))
        return result

    if record:
        capture()
    makespan = simulate(depth)
    return dict(depth=depth, makespan=makespan, decisions=model._variable.decisions,
                candidates=candidates, rollouts=dict(leaves),
                wall_seconds=perf_counter() - started, **({'frames': frames} if record else {}))


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--animate', action='store_true', help='open the top-view event replay')
    parser.add_argument('--depth', type=int, choices=(0, 1, 2), help='choose lookahead depth')
    args = parser.parse_args()
    if args.animate:
        from .nested_crossing_view import show
        results = {depth: run(depth, record=True) for depth in range(3)}
        show(results, 2 if args.depth is None else args.depth)
    else:
        for depth in range(3) if args.depth is None else (args.depth,):
            print(run(depth))


if __name__ == '__main__':
    main()
