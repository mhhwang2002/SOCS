"""Cycle-time tables measured by executing the single-crane AS/RS DEVS model."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.SimEngine import ExecutionResult, SimEngineBase

from .asrs_devs import (
    AISLE_Y,
    ASRSCell,
    ASRSController,
    CELL_WIDTH,
    Crane,
    FORK_APPROACH_Y,
    FORK_RETRACT_Y,
    HOME_X,
    HOME_Z,
    LEVEL_PITCH,
    LOAD_CENTER_OFFSET,
    LOAD_HEIGHT,
    MAX_FORK_REACH,
    RACK_COLUMNS,
    RACK_LEVELS,
    UNLOAD_REACH,
    UNLOAD_STATION_XS,
)


TABLE_FORMAT = "socs-asrs-crane-cycle-times"
TABLE_VERSION = 1


def _default_motion_parameters() -> dict:
    return {
        f"{axis}_{parameter}": getattr(Crane, f"{axis}_{parameter}")
        for axis in ("x", "z", "fork")
        for parameter in ("max_speed", "acceleration", "deceleration")
    }


def _model_signature(motion_parameters=None) -> dict:
    parameters = _default_motion_parameters()
    parameters.update(motion_parameters or {})
    return {
        "rack": {"columns": RACK_COLUMNS, "levels": RACK_LEVELS},
        "geometry": {
            "cell_width": CELL_WIDTH,
            "level_pitch": LEVEL_PITCH,
            "aisle_y": AISLE_Y,
            "home_x": HOME_X,
            "home_z": HOME_Z,
            "max_fork_reach": MAX_FORK_REACH,
            "fork_retract_y": FORK_RETRACT_Y,
            "fork_approach_y": FORK_APPROACH_Y,
            "unload_reach": UNLOAD_REACH,
            "load_height": LOAD_HEIGHT,
            "load_center_offset": LOAD_CENTER_OFFSET,
            "unload_station_xs": list(UNLOAD_STATION_XS),
        },
        "startup_delay": ASRSController.STARTUP_DELAY,
        "crane_parameters": {
            name: {axis: parameters[f"{axis}_{name}"] for axis in ("x", "z", "fork")}
            for name in ("max_speed", "acceleration", "deceleration")
        },
    }


@dataclass(frozen=True)
class CycleDuration:
    """Measured times from command acceptance to box-out and crane-ready."""

    box_out: float
    ready: float


@dataclass(frozen=True)
class CraneCycleTimeTable:
    """Measured single-item cycle durations for every cell and station pair."""

    first_cycles: dict[str, CycleDuration]
    cycles: dict[str, CycleDuration]
    startup_delay: float
    columns: int = RACK_COLUMNS
    levels: int = RACK_LEVELS
    crane_parameters: dict | None = None

    @staticmethod
    def _first_key(bay: int, level: int, end_station: int) -> str:
        return f"{bay}:{level}:{end_station}"

    @staticmethod
    def _cycle_key(start_station: int, bay: int, level: int, end_station: int) -> str:
        return f"{start_station}:{bay}:{level}:{end_station}"

    def duration(
        self,
        start_station: int | None,
        bay: int,
        level: int,
        end_station: int,
    ) -> CycleDuration:
        """Return a cycle duration for zero-based product coordinates."""
        if not 0 <= bay < self.columns or not 0 <= level < self.levels:
            raise ValueError("Cell coordinates are outside the cycle-time table.")
        if end_station not in (0, 1) or (start_station is not None and start_station not in (0, 1)):
            raise ValueError("Station indices must be 0 (left) or 1 (right).")
        key = (
            self._first_key(bay + 1, level + 1, end_station)
            if start_station is None
            else self._cycle_key(start_station, bay + 1, level + 1, end_station)
        )
        table = self.first_cycles if start_station is None else self.cycles
        try:
            return table[key]
        except KeyError as exc:
            raise ValueError(f"Cycle-time table has no entry for {key}.") from exc

    def to_dict(self) -> dict:
        return {
            "format": TABLE_FORMAT,
            "version": TABLE_VERSION,
            "rack": {"columns": self.columns, "levels": self.levels},
            "geometry": _model_signature()["geometry"],
            "startup_delay": self.startup_delay,
            "time_origin": "command accepted after controller dispatch",
            "cycle_endpoints": {
                "box_out": "controller receives the unloaded product",
                "ready": "crane reports idle after fork-down",
            },
            "first_cycles": {
                key: {"box_out": value.box_out, "ready": value.ready}
                for key, value in sorted(self.first_cycles.items())
            },
            "cycles": {
                key: {"box_out": value.box_out, "ready": value.ready}
                for key, value in sorted(self.cycles.items())
            },
            "crane_parameters": self.crane_parameters or _model_signature()["crane_parameters"],
        }

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: str | Path) -> "CraneCycleTimeTable":
        path = Path(path)
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("format") != TABLE_FORMAT or payload.get("version") != TABLE_VERSION:
            raise ValueError(f"Unsupported crane cycle-time table: {path}.")
        rack = payload.get("rack", {})
        if rack != {"columns": RACK_COLUMNS, "levels": RACK_LEVELS}:
            raise ValueError("Crane cycle-time table does not match the 15 x 6 AS/RS rack.")
        signature = _model_signature()
        if payload.get("geometry") != signature["geometry"]:
            raise ValueError("AS/RS geometry changed; regenerate the crane cycle-time table.")
        crane_parameters = payload.get("crane_parameters")
        if not isinstance(crane_parameters, dict) or set(crane_parameters) != {
            "max_speed", "acceleration", "deceleration"
        }:
            raise ValueError("Cycle-time table is missing Crane motion parameters.")
        for parameter, values in crane_parameters.items():
            if not isinstance(values, dict) or set(values) != {"x", "z", "fork"}:
                raise ValueError(f"Invalid Crane {parameter} signature in cycle-time table.")
            if any(float(value) <= 0 for value in values.values()):
                raise ValueError(f"Crane {parameter} values must be positive.")
        if payload.get("startup_delay") != signature["startup_delay"]:
            raise ValueError("Controller startup delay changed; regenerate the cycle-time table.")
        return cls(
            first_cycles={key: CycleDuration(**value) for key, value in payload["first_cycles"].items()},
            cycles={key: CycleDuration(**value) for key, value in payload["cycles"].items()},
            startup_delay=float(payload["startup_delay"]),
            columns=rack["columns"],
            levels=rack["levels"],
            crane_parameters=crane_parameters,
        )


def _probe_cycle(
    bay: int,
    level: int,
    end_station: int,
    start_station: int | None,
    motion_parameters=None,
) -> tuple[CycleDuration, float]:
    """Execute one retrieval through the real ASRS cell, controller, and crane."""
    cell_name = f"Cell-B{bay:02d}-L{level}"
    product_id = f"Probe-B{bay:02d}-L{level}"
    cell = ASRSCell(bay, level, True, box_id=product_id, box_color="red")
    crane = Crane(motion_parameters=motion_parameters)
    retrieval_plan = [{
        "cell": cell_name,
        "product_id": product_id,
        "color": "red",
        "order_id": 1,
        "station": end_station,
    }]
    controller = ASRSController(
        (cell_name,), retrieval_plan=retrieval_plan, requested_order_items={1: 1}
    )
    model = CoupledDEVS("ASRSCycleProbe")
    model.add_child(cell)
    model.add_child(crane)
    model.add_child(controller)
    model.addCouple((crane, "!retrieve_request"), (cell, "?crane"))
    model.addCouple((cell, "!box"), (crane, "?box"))
    model.addCouple((cell, "!status"), (controller, "?cell_status"))
    model.addCouple((controller, "!command"), (crane, "?command"))
    model.addCouple((crane, "!idle"), (controller, "?crane_idle"))
    model.addCouple((crane, "!box_out"), (controller, "?box_out"))

    engine = SimEngineBase(
        model,
        TimeIntervalContinuous=float("inf"),
        TimeIntervalAnimation=float("inf"),
        TimeSimulationEnd=1e6,
    )
    engine.bVerbose = False
    if start_station is None:
        crane.x, crane.y, crane.z = HOME_X, FORK_RETRACT_Y, HOME_Z
    else:
        crane.x = UNLOAD_STATION_XS[start_station]
        crane.y = AISLE_Y + UNLOAD_REACH
        crane.z = HOME_Z
    crane.fork_lift = 0.0

    box_out_at = None
    steps = 0
    while controller.phase != "completed":
        result = engine.executeOneStep()
        steps += 1
        if result is ExecutionResult.TimeBoundReached:
            raise RuntimeError(f"Cycle probe did not complete for {cell_name}.")
        if steps > 1000:
            raise RuntimeError(f"Cycle probe exceeded its event limit for {cell_name}.")
        if 1 in controller.completed_order_times:
            box_out_at = controller.completed_order_times[1]

    if box_out_at is None:
        raise RuntimeError(f"Cycle probe produced no box-out time for {cell_name}.")
    startup_delay = ASRSController.STARTUP_DELAY
    duration = CycleDuration(
        box_out=float(box_out_at - startup_delay),
        ready=float(engine.tNow - startup_delay),
    )
    return duration, startup_delay


def build_cycle_time_table(motion_parameters=None) -> CraneCycleTimeTable:
    """Measure initial and repeated cycles for every cell and L/R station pair."""
    first_cycles = {}
    cycles = {}
    startup_delay = None
    for level in range(1, RACK_LEVELS + 1):
        for bay in range(1, RACK_COLUMNS + 1):
            for end_station in (0, 1):
                duration, delay = _probe_cycle(bay, level, end_station, None, motion_parameters)
                startup_delay = delay
                first_cycles[CraneCycleTimeTable._first_key(bay, level, end_station)] = duration
            for start_station in (0, 1):
                for end_station in (0, 1):
                    duration, delay = _probe_cycle(
                        bay, level, end_station, start_station, motion_parameters
                    )
                    startup_delay = delay
                    cycles[CraneCycleTimeTable._cycle_key(
                        start_station, bay, level, end_station
                    )] = duration
    return CraneCycleTimeTable(
        first_cycles,
        cycles,
        float(startup_delay),
        crane_parameters=_model_signature(motion_parameters)["crane_parameters"],
    )


def main(argv=None):
    import argparse

    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path,
        default=root / "examples" / "ASRS" / "data" / "crane_cycle_times.json",
    )
    parser.add_argument(
        "--motion-parameters", type=Path,
        help="JSON file with flat x/z/fork max-speed, acceleration, and deceleration overrides",
    )
    args = parser.parse_args(argv)
    motion_parameters = (
        json.loads(args.motion_parameters.read_text(encoding="utf-8"))
        if args.motion_parameters
        else None
    )
    table = build_cycle_time_table(motion_parameters)
    path = table.save(args.output)
    print(f"Saved {len(table.first_cycles)} initial and {len(table.cycles)} repeated cycle times to {path}")
    print(f"Controller startup delay excluded from table: {table.startup_delay:g} simulation units")


if __name__ == "__main__":
    main()
