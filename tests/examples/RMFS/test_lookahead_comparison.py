"""First-witness baseline is retained in the finite candidate comparison."""

import pytest

from examples.RMFS.Tests.run_lookahead_comparison import compare_example


@pytest.mark.parametrize('name,improves', [
    ('02_01_RobotPath_Crossing_NoControl', True),
    ('02_09_RobotPath_FirstMoverMatters', True),
    ('02_10_RobotPath_AlternatingProgress', False),
])
def test_fixed_paths_baseline_and_selected_both_validate(name, improves):
    row = compare_example(name)
    assert row['status'] == 'completed'
    assert row['results'][0]['candidate'] == 'first_valid'
    assert row['lookahead_makespan'] <= row['baseline_makespan']
    assert (row['improvement_percent'] > 1e-8) == improves
    assert not row['paths_changed'] and row['initial_snapshot_time'] == 0
    assert row['merge_sweeps'] and row['event_release'] and row['stop_on_collision']
    assert all(v['completed'] and not v['contacts'] for v in row['validations'].values())


def test_one_candidate_budget_reproduces_baseline():
    row = compare_example('02_01_RobotPath_Crossing_NoControl', budget=1)
    assert row['branches'] == 1 and row['selected_index'] == 0
    assert row['baseline_makespan'] == row['lookahead_makespan']


def test_no_plan_is_not_reported_as_successful_completion():
    row = compare_example('02_05_RobotPath_OppositeDirection_Collision')
    assert row['status'] == 'no_plan_in_finite_model'
    assert row['baseline_makespan'] is row['lookahead_makespan'] is None
