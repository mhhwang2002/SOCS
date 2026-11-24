"""Audit actual Push/Pull choices in a saved twenty-robot warehouse workload."""

from argparse import ArgumentParser
from collections import Counter
from itertools import combinations
from json import loads
from math import atan2, degrees, isfinite
from types import MethodType

from socs.devs.TESEngin import TESEnginBase

from ..controlled import create_controlled_model
from ..MotionEnergyCollector import attach_motion_collector
from ..motion import Limits
from ..path_geometry import sweep_2d
from .incremental_routing import dag_witness
from .run_crossing_energy_comparison import DEFAULT_CONFIG, fleet_energy
from .run_warehouse_paths import DATA, load_layout


def run(case, config, *, stop_first=False):
    grid = load_layout()
    grid['warehouse_empty_cells'] = tuple(map(tuple, case['empty_rack_cells']))
    paths = {name: tuple(map(tuple, route)) for name, route in case['routes'].items()}
    headings = {name: degrees(atan2(b[1] - a[1], b[0] - a[0]))
                for name, (a, b, *_) in paths.items()}
    model, robots, _ = create_controlled_model(
        paths, grid=grid, lift_up=True, initial_headings=headings,
        collision_check=True, merge_sweeps=True, event_release=True, safe_prefix=True,
        passage_push=not stop_first, motion_extension=not stop_first,
        motion_limits=Limits(**config['with_rack']),
        plan_builder=lambda solver: dag_witness(solver, case['requirements']))
    manager = model.controller.traffic_manager
    collector = attach_motion_collector(model, robots)
    original = manager.reconsider
    decision_events = set()

    def audited_reconsider(self, time, *, requester=None, cleared_candidates=None):
        candidates = {}
        for index, (_, name, part) in enumerate(self.tasks):
            if (name in candidates or index in self.issued_tasks or name in self.orienting
                    or any(k not in self.finished_tasks and k not in self.command_tasks.get(name, ())
                           for k, (_, owner, _) in enumerate(self.tasks[:index]) if owner == name)):
                continue
            if name in self.active_tasks:
                if requester is not None and requester != name:
                    continue
                extension = self._released_extension(name, index)
                if extension is None:
                    continue
                candidates[name] = sweep_2d(extension[0], self.footprints[name], self.policy)
            elif self._can_start(index):
                candidates[name] = self.task_sweeps[index]
        for a, b in combinations(candidates, 2):
            if candidates[a].intersects(candidates[b], self.policy.tolerance):
                decision_events.add((time, requester, *sorted((a, b))))
        original(time, requester=requester, cleared_candidates=cleared_candidates)

    if not stop_first:
        manager.reconsider = MethodType(audited_reconsider, manager)
    engine = TESEnginBase(model, TimeIntervalAnimation=.01)
    engine.bVerbose = False
    for _ in range(100000):
        if not isfinite(model.tNext):
            break
        engine.execute_one_step()
    else:
        raise RuntimeError('Warehouse audit exceeded its event budget.')
    if not manager.done or model.collision_history:
        raise RuntimeError('Warehouse run did not complete safely.')
    counts = Counter(event['event'] for event in manager.roadmap.history)
    arrivals, energy = fleet_energy(config, collector, paths, loaded=True)
    return dict(case=case['index'] + 1, seed=case['seed'], stop_first=stop_first,
                makespan=max(arrivals.values()),
                intermediate_stops=counts['arrived'] - len(arrivals),
                fleet_energy_wh=sum(energy.values()), contacts=len(model.collision_history),
                push_requests=counts['passage_request'], push_extensions=counts['push_extension'],
                pull_extensions=counts['extension_reserved'] - counts['push_extension'],
                conflicting_eligible_pairs=sorted(decision_events, key=lambda item: (item[0], str(item[1]), item[2:])))


if __name__ == '__main__':
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=int, default=1)
    parser.add_argument('--stop-first', action='store_true')
    args = parser.parse_args()
    case = next((row for row in loads(DATA.read_text(encoding='utf-8'))['cases']
                 if row['index'] + 1 == args.case), None)
    if case is None:
        parser.error('No such saved case')
    print(run(case, loads(DEFAULT_CONFIG.read_text(encoding='utf-8-sig')), stop_first=args.stop_first))
