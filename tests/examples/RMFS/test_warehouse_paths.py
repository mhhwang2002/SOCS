import json
from pathlib import Path

import pytest

from examples.RMFS.WarehouseLayout import load_layout
from examples.RMFS.Tests.run_warehouse_paths import DATA,generate_case
from examples.RMFS.Tests.generate_path_data import measure
from examples.RMFS.path_geometry import Path as MotionPath,GeometryPolicy,RectangularFootprint


def test_archived_layout_geometry_is_self_contained():
    layout=load_layout()
    assert len(layout['control_points'])==401
    assert len(layout['segments'])==527
    assert len(layout['warehouse_racks'])==120
    assert len(layout['warehouse_stations'])==10
    for a,b in layout['segments']:
        assert a[0]==b[0] or a[1]==b[1]
        assert a in layout['adjacency'][b] and b in layout['adjacency'][a]


@pytest.mark.parametrize('index',range(5))
def test_twenty_loaded_routes_have_requested_endpoints_and_metrics(index):
    layout=load_layout();case=json.loads(DATA.read_text())['cases'][index]
    assert len(case['routes'])==20 and len(case['empty_rack_cells'])==20
    roles=[a['role'] for a in case['assignments'].values()]
    assert roles.count('rack_to_station')==roles.count('station_to_rack')==10
    paths={}
    for name,points in case['routes'].items():
        assignment=case['assignments'][name]
        assert assignment['lift_up']
        route=tuple(map(tuple,points));paths[name]=MotionPath(name,route)
        assert len(route)==len(set(route))
        assert all(b in layout['adjacency'][a] for a,b in zip(route,route[1:]))
        station=layout['warehouse_stations'][assignment['station']]
        rack=layout['warehouse_racks'][tuple(assignment['rack_cell'])]
        assert (route[0],route[-1])==((rack,station) if assignment['role']=='rack_to_station' else (station,rack))
    metrics=measure(paths,RectangularFootprint(1.5,1.5),GeometryPolicy())
    assert json.loads(json.dumps(metrics))==case['metrics']
    for i in range(10):
        assert f'R{i+1:02}' in case['requirements'][f'R{i+11:02}']


def test_seed_reproduces_paths_without_policy_performance_filter():
    original=json.loads(DATA.read_text())['cases'][0]
    generated=generate_case(0,20261120)
    assert json.loads(json.dumps(generated))==original
