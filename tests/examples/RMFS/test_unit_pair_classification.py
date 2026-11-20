"""Endpoint classes and the distinction between progress and common-route units."""
from itertools import combinations
import pytest
from examples.RMFS.GridPath import CONTROL_POINTS
from examples.RMFS.path_geometry import Path, PathPosition, RectangularFootprint, interference_2d
from examples.RMFS.wait_duplication import WaitDuplicationSolver
from examples.RMFS.Tests.run_unit_pair_classification import safe_order

EXAMPLES = [((0, 0, 0, 0), ((0, 0), (1, 0)), ((0, 1), (1, 1))), ((0, 0, 0, 1), ((0, 0), (1, 0), (2, 0)), ((0, 1), (1, 1), (1, 0))), ((0, 0, 1, 0), ((0, 0), (1, 0), (2, 0)), ((1, 0), (1, 1))), ((0, 0, 1, 1), ((0, 0), (1, 0), (2, 0), (3, 0)), ((1, 0), (2, 0))), ((0, 1, 0, 0), ((0, 0), (1, 0)), ((0, 1), (1, 1), (1, 0), (2, 0))), ((0, 1, 0, 1), ((0, 0), (1, 0)), ((0, 1), (1, 1), (1, 0))), ((0, 1, 1, 0), ((0, 0), (1, 0)), ((1, 0), (2, 0))), ((0, 1, 1, 1), ((0, 0), (1, 0), (2, 0)), ((1, 0), (2, 0))), ((1, 0, 0, 0), ((0, 0), (1, 0), (2, 0)), ((0, 1), (0, 0), (1, 0), (1, 1))), ((1, 0, 0, 1), ((0, 0), (1, 0)), ((0, 1), (0, 0))), ((1, 0, 1, 0), ((0, 0), (1, 0), (2, 0)), ((1, 0), (0, 0), (0, 1))), ((1, 0, 1, 1), ((0, 0), (1, 0), (2, 0)), ((1, 0), (0, 0))), ((1, 1, 0, 0), ((0, 0), (1, 0)), ((0, 1), (0, 0), (1, 0), (2, 0))), ((1, 1, 0, 1), ((0, 0), (1, 0)), ((0, 1), (0, 0), (1, 0))), ((1, 1, 1, 0), ((0, 0), (1, 0)), ((1, 0), (0, 0), (0, 1))), ((1, 1, 1, 1), ((0, 0), (1, 0)), ((1, 0), (0, 0)))]

@pytest.mark.parametrize("flags,a,b",EXAMPLES)
def test_representative_of_every_endpoint_class(flags,a,b):
    paths={name:Path(name,tuple(CONTROL_POINTS[p] for p in route)) for name,route in (("i",a),("j",b))}
    observed=[]
    for name,other in (("i","j"),("j","i")):
        p=paths[name]
        c=interference_2d(p,RectangularFootprint(),{other:(paths[other],RectangularFootprint())})
        observed.extend((int(c.contains(PathPosition(0))),int(c.contains(PathPosition(len(p.vertices)-1)))))
    assert tuple(observed)==flags
    solver=WaitDuplicationSolver(paths);result=solver.solve()
    assert (result.status=="feasible")==bool(safe_order(flags))
    if result.tuple_path:assert solver.validate(result)

def test_progress_connected_is_not_the_same_as_common_route_connected():
    a=((1,1),(2,1),(2,2),(1,2))
    b=((2,2),(1,2),(1,1),(2,1))
    paths={name:Path(name,tuple(CONTROL_POINTS[p] for p in route)) for name,route in (("i",a),("j",b))}
    for name,other in (("i","j"),("j","i")):
        c=interference_2d(paths[name],RectangularFootprint(),{other:(paths[other],RectangularFootprint())})
        assert len(c.intervals)==1
        assert c.contains(PathPosition(0)) and c.contains(PathPosition(3))
    common_edges={frozenset(edge) for edge in zip(a,a[1:])}&{frozenset(edge) for edge in zip(b,b[1:])}
    assert len(common_edges)==2
    assert not set.intersection(*(set(e) for e in common_edges))
    solver=WaitDuplicationSolver(paths);result=solver.solve()
    assert result.status=="feasible" and solver.validate(result)
