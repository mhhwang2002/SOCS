import json
from math import inf,isfinite
from pathlib import Path

from examples.RMFS.Tests.run_generated_policy import factory
from examples.RMFS.Tests import run_unified_policy as unified
from examples.RMFS.path_geometry import Subpath,sweep_2d
from socs.devs.TESEngin import TESEnginBase


def test_saved_paths_execute_without_changing_geometry():
    source = Path(__file__).resolve().parents[3]/'examples/RMFS/Tests/data/generated/random_trial.json'
    data = json.loads(source.read_text(encoding='utf-8'))
    build, count = factory(data,0,budget=3)
    model = build('sample_1','fifo')
    sample = data['samples'][0]
    assert count >= 1
    for name,path in model.controller.roadmap.paths.items():
        assert path.vertices == tuple(map(tuple,sample['paths_m'][name]))
    for policy in ('fifo','length','lookahead_makespan'):
        trial = unified.run('sample_1',policy,model_factory=build)
        assert trial['completed']
        assert unified.validate(trial,model_factory=build)['contacts'] == 0
        if policy=='lookahead_makespan':
            assert trial['wait_decisions'] > 0
        else:
            assert trial['wait_decisions'] == 0  # counts external lookahead calls only
        assert trial['execution_trace']['wait_actions']
        for decision in trial['decisions']:
            feasible = [b for b in decision['branches'] if b['completed']]
            assert trial['makespan'] <= min(b['makespan'] for b in feasible)+1e-9


def test_optional_wait_needs_residual_interference_and_committed_arrival():
    source=Path(__file__).resolve().parents[3]/'examples/RMFS/Tests/data/generated/random_trial.json'
    data=json.loads(source.read_text(encoding='utf-8'))
    build,_=factory(data,0,budget=3)
    engine=TESEnginBase(build('sample_1','lookahead_makespan'),
                       TimeIntervalContinuous=inf,TimeIntervalAnimation=inf)
    engine.bVerbose=False
    manager=engine.model.controller.traffic_manager
    for _ in range(2000):
        if manager.pending_wait: break
        assert isfinite(engine.model.tNext)
        engine.execute_one_step()
    option=manager.pending_wait
    assert option
    a,b=option['a'],option['b']
    approach=manager.roadmap.robots[b]['approved_path']
    assert b in manager.active_tasks and approach.end==option['gb']
    point=sweep_2d(Subpath(manager.roadmap.paths[a],option['ga'],option['ga']),
                   manager.footprints[a],manager.policy)
    assert not point.intersects(sweep_2d(approach,manager.footprints[b],manager.policy),manager.policy.tolerance)
    index=next(i for i,(_,n,p) in enumerate(manager.tasks)
               if n==a and p.start==option['ga'] and i not in manager.issued_tasks)
    assert manager._approach_option(index,engine.tNow)
    edges=manager.roadmap.interference_edges
    manager.roadmap.interference_edges=()
    assert manager._approach_option(index,engine.tNow) is None
    manager.roadmap.interference_edges=edges
    active=manager.active_tasks.pop(b)
    assert manager._approach_option(index,engine.tNow) is None
    manager.active_tasks[b]=active
    snapshot=engine.capture_snapshot()
    unified.apply(engine,'wait_or_go','wait_for_B')
    assert a in manager.residual_holds
    engine.restore_snapshot(snapshot)
    assert manager.pending_wait==option and not manager.residual_holds
