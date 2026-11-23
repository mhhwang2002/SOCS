"""Large load fixtures use factored references, not an exponential portfolio."""

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.Tests import run_large_policy as large
from examples.RMFS.Tests import run_unified_policy as unified


def test_20_by_20_fixture_geometry_capacity_and_local_reference_storage():
    routes,grid,pairs,_=large.fixture(24)
    assert len(grid['control_points'])==400 and len(routes)==24 and len(pairs)==12
    assert len({route[0] for route in routes.values()})==24
    assert len({route[-1] for route in routes.values()})==24
    assert all(len(route)==len(set(route)) for route in routes.values())
    model=large.build('grid20_24','fifo')
    assert model.scaling_metadata['stored_reference_options']==24
    assert model.scaling_metadata['represented_reference_combinations']==4096
    assert len(model.controller.traffic_manager.plan_predecessors)==24


def test_factored_snapshot_retains_at_least_one_option_for_every_group():
    model=large.build('grid20_12','lookahead_makespan')
    engine=TESEnginBase(model)
    engine.bVerbose=False
    manager=model.controller.traffic_manager
    snapshot=engine.capture_snapshot()
    before=set(manager.viable_plans)
    assert all(before & options for options in manager.task_options.values())
    manager.viable_plans.clear()
    engine.restore_snapshot(snapshot)
    assert manager.viable_plans==before


def test_twelve_objects_have_real_decisions_and_complete_without_nested_search():
    row=unified.run('grid20_12','lookahead_makespan',model_factory=large.build)
    assert row['completed'] and row['lookahead_calls']>=6
    assert row['wait_decisions']>=6
    # The chosen real actions may all be immediate. Wait alternatives must
    # nevertheless encounter and resolve ordinary waiting contention.
    assert any(branch['execution_trace']['waiting_choices']
               for decision in row['decisions'] for branch in decision['branches'])
    assert row['rollouts']>=8*row['wait_decisions']
    assert not row['nested_lookahead']
    assert row['total_seconds']>=row['wall_seconds']>=row['decision_seconds']
    assert len(row['completion_times'])==12
