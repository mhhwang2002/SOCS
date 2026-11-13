"""Compare cycle-table online replay against the live SOCS DEVS AS/RS model."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

from socs.devs.TESEngin import ExecutionResult, TESEnginBase

from .aco_optimization import COLORS, load_instance
from .asrs_devs import ASRSController, make_online_asrs_model
from .crane_cycle_times import CraneCycleTimeTable
from .online_dispatch_experiment import _trace_read, simulate_online_policy


VALIDATED_POLICIES = ("FCFS", "Random", "EDD", "minimum_slack", "nearest_pick")


def _inventory(instance):
    return {
        (product.bay + 1, product.level + 1): {
            "box_id": product.product_id,
            "color": product.color,
        }
        for product in instance.products
    }


def _motion_parameters(table):
    return {
        f"{axis}_{parameter}": value
        for parameter, axis_values in table.crane_parameters.items()
        for axis, value in axis_values.items()
    }


def _execute_devs(instance, trace, table, policy, seed):
    model = make_online_asrs_model(
        _inventory(instance),
        trace["orders"],
        table,
        dispatch_policy=policy,
        random_seed=seed,
        motion_parameters=_motion_parameters(table),
    )
    controller = next(child for child in model.children if isinstance(child, ASRSController))
    engine = TESEnginBase(
        model,
        TimeIntervalContinuous=float("inf"),
        TimeIntervalAnimation=float("inf"),
        TimeSimulationEnd=1e9,
    )
    engine.bVerbose = False
    steps = 0
    while controller.phase != "completed":
        result = engine.executeOneStep()
        steps += 1
        if result is ExecutionResult.TimeBoundReached:
            raise RuntimeError("The dynamic DEVS AS/RS model did not finish its arrival trace.")
        if steps > 100_000:
            raise RuntimeError("The dynamic DEVS AS/RS model exceeded its event-step limit.")
    return controller, float(engine.tNow)


def validate_trace(instance_path: Path, trace_path: Path, table_path: Path,
                   policies=VALIDATED_POLICIES, seed=42,
                   allow_reference_table_mismatch=False):
    instance_path = Path(instance_path)
    if instance_path.is_dir():
        trace_header = json.loads(trace_path.read_text(encoding="utf-8"))
        expected_hash = trace_header.get("source_instance", {}).get("sha256")
        matching_paths = [
            path for path in instance_path.glob("*.json")
            if hashlib.sha256(path.read_bytes()).hexdigest() == expected_hash
        ]
        if len(matching_paths) != 1:
            raise ValueError(f"Could not resolve one source instance for {trace_path}.")
        instance_path = matching_paths[0]
    trace, instance = _trace_read(trace_path, instance_path)
    table = CraneCycleTimeTable.load(table_path)
    if (
        not allow_reference_table_mismatch
        and hashlib.sha256(table_path.read_bytes()).hexdigest()
        != trace["generation"]["cycle_time_table_sha256"]
    ):
        raise ValueError("The trace's deadline-generation table differs from the validation table.")

    rows = []
    for policy_index, policy in enumerate(policies):
        policy_seed = seed + policy_index * 997
        replay = simulate_online_policy(
            instance,
            trace,
            table,
            policy,
            seed=policy_seed,
            capture_log=True,
        )
        controller, end_time = _execute_devs(instance, trace, table, policy, policy_seed)
        expected = replay["pick_log"]
        observed = controller.box_out_log
        observed_dispatches = controller.dispatch_log
        if len(expected) != len(observed) or len(expected) != len(observed_dispatches):
            raise AssertionError(
                f"{trace_path.name}/{policy}: replay produced {len(expected)} picks, "
                f"DEVS produced {len(observed)} box-outs and {len(observed_dispatches)} dispatches."
            )

        identity_mismatches = []
        max_dispatch_error = 0.0
        max_box_out_error = 0.0
        for expected_pick, observed_pick, observed_dispatch in zip(
            expected, observed, observed_dispatches
        ):
            expected_identity = (
                expected_pick["order_id"], expected_pick["box"],
                expected_pick["cell"], expected_pick["station"],
            )
            observed_identity = (
                observed_pick.get("order_id"), observed_pick.get("box"),
                observed_pick.get("cell"), observed_pick.get("station"),
            )
            if expected_identity != observed_identity:
                identity_mismatches.append((expected_identity, observed_identity))
            max_dispatch_error = max(
                max_dispatch_error,
                abs(float(expected_pick["dispatch_time"]) - float(observed_dispatch["time"])),
            )
            max_box_out_error = max(
                max_box_out_error,
                abs(float(expected_pick["completed_at"]) - float(observed_pick["completed_at"])),
            )

        if identity_mismatches:
            raise AssertionError(
                f"{trace_path.name}/{policy}: replay and DEVS selected different tasks; "
                f"first mismatch={identity_mismatches[0]}"
            )
        if max_dispatch_error > 1e-8 or max_box_out_error > 1e-8:
            raise AssertionError(
                f"{trace_path.name}/{policy}: timing mismatch; dispatch={max_dispatch_error}, "
                f"box_out={max_box_out_error}"
            )

        replay_completion = replay["full_run_makespan"]
        devs_completion = max(
            (float(value) for value in controller.completed_order_times.values()),
            default=0.0,
        )
        rows.append({
            "trace": trace_path.stem,
            "policy": policy,
            "picks": len(expected),
            "identity_match": True,
            "max_dispatch_time_error": max_dispatch_error,
            "max_box_out_time_error": max_box_out_error,
            "replay_makespan": replay_completion,
            "devs_makespan": devs_completion,
            "makespan_error": abs(replay_completion - devs_completion),
            "devs_end_time_after_fork_down": end_time,
        })
    return rows


def run_validation(instance_path: Path, trace_dir: Path, table_path: Path,
                   output_dir: Path, seed=42, policies=VALIDATED_POLICIES,
                   allow_reference_table_mismatch=False):
    traces = sorted(trace_dir.glob("*.json"))
    if not traces:
        raise ValueError(f"No JSON traces found in {trace_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for trace_path in traces:
        rows.extend(validate_trace(
            instance_path,
            trace_path,
            table_path,
            policies,
            seed,
            allow_reference_table_mismatch,
        ))
    raw_path = output_dir / "online_devs_replay_comparison.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "instances": str(instance_path),
        "arrival_traces": [str(path) for path in traces],
        "cycle_time_table": str(table_path),
        "cycle_time_table_sha256": hashlib.sha256(table_path.read_bytes()).hexdigest(),
        "allow_deadline_reference_table_mismatch": allow_reference_table_mismatch,
        "validated_policies": list(policies),
        "trace_policy_pairs": len(rows),
        "identity_match_for_all_pairs": all(row["identity_match"] for row in rows),
        "maximum_dispatch_time_error": max(row["max_dispatch_time_error"] for row in rows),
        "maximum_box_out_time_error": max(row["max_box_out_time_error"] for row in rows),
        "maximum_makespan_error": max(row["makespan_error"] for row in rows),
        "note": "Dynamic order arrivals and dispatch run through the SOCS DEVS source, controller, cells, and crane. Replay and DEVS share the same table-based greedy dispatch decisions; this verifies event integration and Crane timing, not independent policy optimality.",
    }
    report_path = output_dir / "online_devs_replay_comparison.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Validated {len(rows)} trace-policy pairs: {report_path.resolve()}")
    print(f"  maximum dispatch timing error: {report['maximum_dispatch_time_error']:.3e}")
    print(f"  maximum box-out timing error: {report['maximum_box_out_time_error']:.3e}")
    print(f"  maximum makespan error: {report['maximum_makespan_error']:.3e}")


def main(argv=None):
    default_table = Path(__file__).resolve().parent / "data" / "crane_cycle_times.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", type=Path, required=True)
    parser.add_argument("--trace-dir", type=Path, required=True)
    parser.add_argument("--cycle-times", type=Path, default=default_table)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--policies", nargs="+", choices=VALIDATED_POLICIES, default=VALIDATED_POLICIES)
    parser.add_argument(
        "--allow-reference-table-mismatch", action="store_true",
        help="keep trace arrival/deadline times fixed while validating another DEVS Crane profile",
    )
    args = parser.parse_args(argv)
    run_validation(
        args.instance,
        args.trace_dir,
        args.cycle_times,
        args.output_dir,
        args.seed,
        tuple(args.policies),
        args.allow_reference_table_mismatch,
    )


if __name__ == "__main__":
    main()
