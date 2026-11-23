"""Timed alternatives keep fixed geometry and use the existing Plant controller."""

import pytest

from examples.RMFS.Tests.run_priority_lookahead import experiment, ordered_witness
from examples.RMFS.path_geometry import Path
from examples.RMFS.wait_duplication import WaitDuplicationSolver


@pytest.mark.parametrize('case,winner', [
    ('long_horizontal', ['R1', 'R2']),
    ('long_vertical', ['R2', 'R1']),
])
def test_snapshot_priority_selection_and_collision_stopping(case, winner):
    first = experiment(case)
    second = experiment(case)
    assert first['selected'] == second['selected'] == winner
    assert first['snapshot_time'] == 0
    assert first['evaluation_collision_check'] is False
    assert first['validation']['completed'] and first['validation']['contacts'] == 0
    assert first['stop_on_collision']
    a, b = first['branches']
    assert all(branch['completed'] for branch in (a, b))
    assert abs(a['makespan']-b['makespan']) > 2
    assert first['selected_makespan'] == min(a['makespan'], b['makespan'])
    assert [(x['order'],x['makespan'],x['commands']) for x in first['branches']] == [
        (x['order'],x['makespan'],x['commands']) for x in second['branches']]


def test_reference_order_cannot_override_geometric_feasibility():
    solver = WaitDuplicationSolver({
        'R1': Path('R1', ((0.,0.), (2.,0.))),
        'R2': Path('R2', ((2.,0.), (0.,0.))),
    })
    # Reference order alone is no license to pass through an idle footprint.
    with pytest.raises(ValueError):
        ordered_witness(solver, ('R1','R2'))
    with pytest.raises(ValueError):
        ordered_witness(solver, ('R2','R1'))
