"""Run original 17 route cases through the real controlled DEVS plant.

No-plan cases are rejected before any plant command. Default verification uses
external 0.01 s animation samples for collision checks, not controller polling.
"""

import argparse
import importlib
import json
from math import inf, isfinite
from pathlib import Path

from socs.devs.TESEngin import TESEnginBase

from ..GridPath import CONTROL_POINTS
from ..controlled import create_controlled_model


def run_all(*, events_only=False):
    results = []
    for source in sorted(Path(__file__).parent.glob("0[23]_*.py")):
        if "_Controlled" in source.stem:
            continue
        module = importlib.import_module(f"examples.RMFS.Tests.{source.stem}")
        try:
            model, robots, sensors = create_controlled_model(
                module.PATHS, initial_headings=getattr(module, "INITIAL_HEADINGS", None),
                collision_check=not events_only)
        except ValueError as error:
            if not str(error).startswith("No wait-duplication"):
                raise
            row = dict(example=source.stem, status="no_plan", reason=str(error))
        else:
            engine = TESEnginBase(model, TimeIntervalAnimation=inf if events_only else .01)
            engine.bVerbose = False
            steps = 0
            while isfinite(model.tNext):
                engine.execute_one_step()
                steps += 1
                if steps >= 100000:
                    raise RuntimeError("Finite controlled example did not terminate.")
            manager = model.controller.traffic_manager
            assert manager.done and not manager.awaiting and not manager.command_buffer
            assert not model.collision_history
            for robot, sensor in zip(robots, sensors):
                expected = CONTROL_POINTS[module.PATHS[robot.name][-1]]
                assert robot.phase == "Wait" and robot.position_at(engine.tNow) == expected
                assert sensor.phase == "Idle" and not sensor._variable.pending_events
                assert model.controller.roadmap.robots[robot.name]["reported_position"] == expected
            row = dict(example=source.stem, status="completed", time=engine.tNow,
                       tuple_stages=len(manager.completed_stages),
                       commands=manager.next_command_id, engine_steps=steps,
                       collision_samples_enabled=not events_only,
                       contacts=list(model.collision_history),
                       roadmap_history=model.controller.roadmap.history,
                       stage_history=manager.stage_history)
        results.append(row)
        print(f"{source.stem}: {row['status']}" +
              (f"; t={row['time']:.6f}s; commands={row['commands']}; contacts=0"
               if row["status"] == "completed" else ""))
    print(f"Completed {sum(r['status'] == 'completed' for r in results)}, "
          f"no plan {sum(r['status'] == 'no_plan' for r in results)}")
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--events-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rows = run_all(events_only=args.events_only)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(rows, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
