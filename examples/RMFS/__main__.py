"""Run the RMFS single-order example and print its phase trace."""

from argparse import ArgumentParser
from math import isfinite

from socs.devs.TESEngin import TESEnginBase

from .model import BINS_PER_RACK, CLUSTERS, RACKS_PER_CLUSTER, WORKSTATIONS, OrderAllocator, RMFS


def run_order(assignment):
    print(f"{assignment.rack_id} -> {assignment.workstation}")
    model = RMFS(assignment)
    engine = TESEnginBase(model)
    engine.bVerbose = False
    print(
        f"{CLUSTERS} clusters, {CLUSTERS * RACKS_PER_CLUSTER} racks, "
        f"{BINS_PER_RACK} bins per rack, {WORKSTATIONS} workstations"
    )
    while isfinite(engine.pick_next_execution_time_type()[0]):
        engine.execute_one_step()
        print(
            f"t={engine.tNow:5.1f}  "
            f"Controller={model.controller.phase}  {model.robot.name}={model.robot.phase}"
        )
    if model.controller.phase != "Done" or model.robot.phase != "Done":
        raise RuntimeError("The demo stopped before completing the mission.")


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--orders", type=int, default=1)
    parser.add_argument("--seed", type=int)
    args = parser.parse_args()
    if args.orders < 1:
        parser.error("--orders must be positive")
    allocator = OrderAllocator(args.seed)
    for _ in range(args.orders):
        run_order(allocator.assign())
    print(f"Workstation assignments: {allocator.counts}")


if __name__ == "__main__":
    main()


