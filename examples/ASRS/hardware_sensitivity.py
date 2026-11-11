"""Run online AS/RS policies across explicitly sourced Crane motion profiles."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from statistics import mean

from .aco_optimization import load_instance
from .crane_cycle_times import CraneCycleTimeTable, build_cycle_time_table
from .online_dispatch_experiment import POLICIES, _mean_ci, simulate_online_policy


METRICS = (
    "complete_orders_by_deadline", "rescue_units", "mean_response_time",
    "p95_response_time", "full_run_makespan", "optimizer_seconds",
)


def _load_profile_file(path: Path):
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("format") != "socs-asrs-crane-hardware-scenarios" or payload.get("version") != 1:
        raise ValueError(f"Unsupported hardware scenario file: {path}")
    return payload


def run_sensitivity(
    instances_path: Path,
    trace_dir: Path,
    output_dir: Path,
    scenario_path: Path,
    seed: int = 42,
    repeats: int = 2,
    ants: int = 6,
    iterations: int = 8,
    policies=POLICIES,
):
    instance_paths = (
        sorted(instances_path.glob("*.json"))
        if instances_path.is_dir()
        else [instances_path]
    )
    if not instance_paths:
        raise ValueError(f"No inventory/order instances found in {instances_path}")
    instances_by_hash = {
        hashlib.sha256(path.read_bytes()).hexdigest(): (path, load_instance(path))
        for path in instance_paths
    }
    traces = sorted(trace_dir.glob("*.json"))
    if not traces:
        raise ValueError(f"No arrival traces found in {trace_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    cycle_dir = output_dir / "cycle_tables"
    cycle_dir.mkdir(parents=True, exist_ok=True)
    scenario_file = _load_profile_file(scenario_path)
    profile_tables = []

    for profile in scenario_file["profiles"]:
        table = build_cycle_time_table(profile["parameters"])
        table_path = cycle_dir / f"{profile['id']}.json"
        table.save(table_path)
        profile_tables.append((profile, table, table_path))
        print(f"Built DEVS cycle table for {profile['id']}: {table_path}")

    raw = []
    for trace_index, trace_path in enumerate(traces):
        trace = json.loads(trace_path.read_text(encoding="utf-8"))
        source_hash = trace.get("source_instance", {}).get("sha256")
        if source_hash not in instances_by_hash:
            raise ValueError(f"Trace {trace_path} does not match a provided instance.")
        _, instance = instances_by_hash[source_hash]
        trace_reference_hash = trace["generation"]["cycle_time_table_sha256"]
        for profile, table, table_path in profile_tables:
            evaluation_hash = hashlib.sha256(table_path.read_bytes()).hexdigest()
            for policy in policies:
                policy_repeats = repeats if policy in {"Random", "rolling_ACO"} else 1
                for run in range(policy_repeats):
                    run_seed = seed + trace_index * 10007 + run
                    metrics = simulate_online_policy(
                        instance,
                        trace,
                        table,
                        policy,
                        seed=run_seed,
                        ants=ants,
                        iterations=iterations,
                    )
                    raw.append({
                        "scenario": Path(instances_by_hash[source_hash][0]).stem,
                        "trace": trace_path.stem,
                        "arrival_seed": trace["generation"]["seed"],
                        "profile": profile["id"],
                        "policy": policy,
                        "run": run,
                        "seed": run_seed,
                        "mean_interarrival": trace["generation"]["mean_interarrival_time"],
                        "due_slack_factor": trace["generation"]["due_slack_factor"],
                        "due_date_reference_cycle_table_sha256": trace_reference_hash,
                        "evaluation_cycle_table_sha256": evaluation_hash,
                        **metrics,
                    })

    raw_path = output_dir / "hardware_sensitivity_raw.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(raw[0]))
        writer.writeheader()
        writer.writerows(raw)

    baseline_id = scenario_file["profiles"][0]["id"]
    summary = []
    condition_groups = sorted({
        (row["profile"], float(row["mean_interarrival"]), float(row["due_slack_factor"]), row["policy"])
        for row in raw
    })
    for profile_id, mean_interarrival, due_slack, policy in condition_groups:
        scenario_values = {}
        for scenario_name in sorted({
            row["scenario"] for row in raw
            if row["profile"] == profile_id and row["policy"] == policy
            and float(row["mean_interarrival"]) == mean_interarrival
            and float(row["due_slack_factor"]) == due_slack
        }):
            rows = [
                row for row in raw
                if row["profile"] == profile_id and row["policy"] == policy
                and row["scenario"] == scenario_name
                and float(row["mean_interarrival"]) == mean_interarrival
                and float(row["due_slack_factor"]) == due_slack
            ]
            scenario_values[scenario_name] = {
                metric: mean(float(row[metric]) for row in rows)
                for metric in METRICS
            }
        record = {
            "profile": profile_id,
            "policy": policy,
            "mean_interarrival": mean_interarrival,
            "due_slack_factor": due_slack,
            "scenarios": len(scenario_values),
        }
        for metric in METRICS:
            values = [row[metric] for row in scenario_values.values()]
            average, deviation, low, high = _mean_ci(values)
            record[metric] = average
            record[f"{metric}_scenario_sd"] = deviation
            record[f"{metric}_ci95_low"] = low
            record[f"{metric}_ci95_high"] = high

            baseline_values = {}
            for scenario_name in scenario_values:
                baseline_rows = [
                    row for row in raw
                    if row["profile"] == baseline_id and row["policy"] == policy
                    and row["scenario"] == scenario_name
                    and float(row["mean_interarrival"]) == mean_interarrival
                    and float(row["due_slack_factor"]) == due_slack
                ]
                if baseline_rows:
                    baseline_values[scenario_name] = mean(float(row[metric]) for row in baseline_rows)
            paired_deltas = [
                scenario_values[scenario_name][metric] - baseline_values[scenario_name]
                for scenario_name in scenario_values
                if scenario_name in baseline_values
            ]
            delta, _, delta_low, delta_high = _mean_ci(paired_deltas)
            record[f"{metric}_delta_vs_baseline"] = delta
            record[f"{metric}_delta_vs_baseline_ci95_low"] = delta_low
            record[f"{metric}_delta_vs_baseline_ci95_high"] = delta_high
        summary.append(record)

    summary_path = output_dir / "hardware_sensitivity_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)

    design = {
        "scenario_file": str(scenario_path),
        "source_instances": [
            {"path": str(path), "sha256": digest}
            for digest, (path, _) in instances_by_hash.items()
        ],
        "arrival_traces": [str(path) for path in traces],
        "arrival_deadlines": "held fixed from the baseline-reference traces while the evaluated Crane profile changes",
        "profiles": [
            {
                "id": profile["id"],
                "parameters": profile["parameters"],
                "source_status": profile["source_status"],
                "notes": profile["notes"],
                "table": str(table_path),
                "table_sha256": hashlib.sha256(table_path.read_bytes()).hexdigest(),
            }
            for profile, _, table_path in profile_tables
        ],
        "policies": list(policies),
        "repeats": repeats,
        "ants": ants,
        "iterations": iterations,
        "statistics": "repeated Random/ACO seeds averaged per instance; paired profile deltas use the same instances and fixed traces; intervals across source instances",
        "hardware_warning": "Manufacturer records are preliminary survey values, not independently verified specifications. Results are sensitivity scenarios, not a calibrated hardware prediction.",
    }
    (output_dir / "hardware_sensitivity_design.json").write_text(
        json.dumps(design, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"Saved {len(raw)} hardware sensitivity observations for {len(profile_tables)} profiles to {output_dir.resolve()}")
    for row in summary:
        if row["policy"] == "FCFS":
            print(
                f"{row['profile']} FCFS: mean response={row['mean_response_time']:.2f}, "
                f"P95={row['p95_response_time']:.2f}, "
                f"makespan={row['full_run_makespan']:.2f}"
            )


def main(argv=None):
    default_scenarios = Path(__file__).resolve().parent / "data" / "crane_hardware_scenarios.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instances", type=Path, required=True)
    parser.add_argument("--trace-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenario-file", type=Path, default=default_scenarios)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--ants", type=int, default=6)
    parser.add_argument("--iterations", type=int, default=8)
    args = parser.parse_args(argv)
    run_sensitivity(
        args.instances,
        args.trace_dir,
        args.output_dir,
        args.scenario_file,
        args.seed,
        args.repeats,
        args.ants,
        args.iterations,
    )


if __name__ == "__main__":
    main()
