"""Reproduce the 2x2 comparison twice, checking inputs and effective options."""

import json
from hashlib import sha256
from math import isfinite
from pathlib import Path

from socs.devs.TESEngin import TESEnginBase
from .incremental_routing import create_experiment_model


def run():
    results = []
    expected_inputs = {}
    repeated = {}
    for repetition in range(2):
        for count in (10,20):
            for merged in (False,True):
                for release in (False,True):
                    model,robots,sensors = create_experiment_model(
                        robot_count=count,merge_sweeps=merged,event_release=release,
                        collision_check=True,stop_on_collision=True)
                    manager = model.controller.traffic_manager
                    assert manager.event_release == release
                    assert (model.merge_blocks is not None) == merged
                    assert all(obj.stop_on_collision for obj in (*robots,*sensors))
                    inputs = (sha256(repr(model.controller.roadmap.paths).encode()).hexdigest(),
                              sha256(repr(model.plan_result.tuple_path).encode()).hexdigest())
                    assert manager.tuple_path == model.plan_result.tuple_path
                    assert expected_inputs.setdefault(count,inputs) == inputs
                    initial_commands = [command['robot'] for command in manager.command_buffer]
                    engine = TESEnginBase(model,TimeIntervalAnimation=.01)
                    engine.bVerbose = False
                    iterations = 0
                    while isfinite(model.tNext):
                        engine.execute_one_step()
                        iterations += 1
                        assert iterations < 200000
                    assert manager.done and not model.collision_history
                    assert all(robot.phase=='Wait' and sensor.phase=='Idle'
                               for robot,sensor in zip(robots,sensors))
                    for robot in robots:
                        assert robot.position_at(engine.tNow) == model.controller.roadmap.paths[robot.name].vertices[-1]
                    key = count,merged,release
                    assert repeated.setdefault(key,engine.tNow) == engine.tNow
                    per_robot = {}
                    for robot in robots:
                        history = [e for e in model.controller.roadmap.history if e['robot']==robot.name]
                        issues = [e for e in history if e['event']=='issued']
                        arrivals = [e for e in history if e['event']=='arrived']
                        per_robot[robot.name] = dict(first_issue=issues[0]['time'],
                            final_arrival=arrivals[-1]['time'],commands=len(issues))
                    results.append(dict(repetition=repetition,robots=count,merge_sweeps=merged,
                        event_release=release,makespan=engine.tNow,path_hash=inputs[0],
                        source_plan_hash=inputs[1],control_plan_hash=sha256(repr(manager.tuple_path).encode()).hexdigest(),
                        implementation='original_control_units_with_safe_command_prefix_merging',
                        collision_interval=.01,stop_on_collision=True,
                        contacts=0,completed=True,initial_commands=initial_commands,per_robot=per_robot))
                    print(repetition,count,merged,release,f'{engine.tNow:.6f}',flush=True)
    destination = Path(__file__).parent/'results'/'merge_release_audit.json'
    destination.write_text(json.dumps(results,indent=2),encoding='utf-8')
    print('All repeated results and input hashes agree:',destination)


if __name__ == '__main__':
    run()
