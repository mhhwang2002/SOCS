from dataclasses import replace
import json
import pytest

from examples.RMFS.GridPath import make_grid
from examples.RMFS.motion import LOADED, StraightProfile
from examples.RMFS.motion_graph import compile_motion_graph, save_motion_graph, load_motion_graph


def test_grid_runs_and_uninterrupted_duration():
    graph = compile_motion_graph(make_grid())
    assert graph["counts"] == dict(nodes=16, original_arcs=48,
                                  pass_through_arcs=48, total_arcs=96)
    edge = next(e for e in graph["edges"] if e["source"] == [0, 0] and e["target"] == [3, 0])
    assert edge["path"] == [[0, 0], [1, 0], [2, 0], [3, 0]]
    assert edge["length_m"] == 6
    assert edge["travel_time_s"]["loaded"] == pytest.approx(StraightProfile(6).duration)
    assert edge["travel_time_s"]["loaded"] < 3*StraightProfile(2).duration
    assert not any(e["source"] == [0, 0] and e["target"] == [3, 3] for e in graph["edges"])


def test_directed_gap_and_turn_are_not_shortcuts():
    layout = dict(control_points={"a": (0, 0), "b": (2, 0), "c": (4, 0), "d": (2, 2)},
                  adjacency={"a": ("b",), "b": ("d",), "c": (), "d": ()})
    graph = compile_motion_graph(layout)
    assert {(e["source"], e["target"]) for e in graph["edges"]} == {("a", "b"), ("b", "d")}


def test_vertical_runs_supported_and_diagonals_rejected():
    layout = dict(control_points={0: (0, 0), 1: (0, 1), 2: (0, 3)},
                  adjacency={0: (1,), 1: (2,), 2: ()})
    e = next(e for e in compile_motion_graph(layout)["edges"] if e["target"] == 2 and e["source"] == 0)
    assert e["length_m"] == 3
    assert e["entry_heading_deg"] == pytest.approx(90)
    layout["control_points"][2] = (2, 3)
    with pytest.raises(ValueError, match="horizontal or vertical"):
        compile_motion_graph(layout)


def test_cache_round_trip_and_invalidation(tmp_path):
    layout = make_grid()
    graph = compile_motion_graph(layout)
    p = tmp_path/"graph.json"
    save_motion_graph(graph, p)
    assert load_motion_graph(p, layout=layout) == json.loads(json.dumps(graph))
    with pytest.raises(ValueError, match="stale"):
        load_motion_graph(p, layout=make_grid(spacing_m=3))
    with pytest.raises(ValueError, match="stale"):
        load_motion_graph(p, layout=layout, profiles={"loaded": replace(LOADED, speed=1)})


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError, match="positive"):
        compile_motion_graph(make_grid(), profiles={"loaded": replace(LOADED, acceleration=0)})


def test_macro_edge_keeps_intermediate_clearance_reports(capsys):
    from importlib import import_module
    from math import inf, isfinite
    from socs.devs.TESEngin import TESEnginBase
    graph = compile_motion_graph(make_grid())
    edge = next(e for e in graph["edges"] if e["source"] == [0, 0] and e["target"] == [3, 0])
    path = tuple(tuple(p) for p in edge["path"])
    scene = import_module("examples.RMFS.Tests.01_01_RobotPath_Straight")
    model, robot, sensor = scene.create_model(path=path, lift_up=True)
    engine = TESEnginBase(model, TimeIntervalAnimation=inf)
    engine.bVerbose = False
    steps = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        steps += 1
        assert steps < 100
    text = capsys.readouterr().out
    for x in (.75, 2.75, 4.75):
        assert f"(?R1, ({x}, 0.0))" in text
    assert "?arrived_R1, (6.0, 0.0)" in text
    assert "?rotation_started" not in text
    assert robot._variable.profile.duration == pytest.approx(edge["travel_time_s"]["loaded"])
    assert robot.phase == "Wait" and sensor.phase == "Idle"
