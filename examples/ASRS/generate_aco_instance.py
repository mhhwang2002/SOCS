"""Generate and save a reproducible AS/RS ACO problem instance."""

from __future__ import annotations

import argparse
from pathlib import Path

from .aco_optimization import generate_instance, save_instance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42, help="random seed for problem generation")
    parser.add_argument(
        "--output",
        type=Path,
        help="JSON output path (defaults to examples/ASRS/instances/asrs_seed_<seed>.json)",
    )
    args = parser.parse_args(argv)

    output = args.output
    if output is None:
        project_root = Path(__file__).resolve().parents[2]
        output = project_root / "examples" / "ASRS" / "instances" / f"asrs_seed_{args.seed}.json"
    elif not output.is_absolute():
        output = Path.cwd() / output

    instance = generate_instance(args.seed)
    saved_path = save_instance(instance, output, seed=args.seed)
    print(f"Saved AS/RS problem instance: {saved_path.resolve()}")
    print(f"Products: {len(instance.products)}; orders: {len(instance.order_products)}")
    for order_id, product_ids in enumerate(instance.order_products, start=1):
        shortage = instance.order_penalties[order_id - 1]
        requested = len(product_ids) + shortage
        demand = instance.order_demands[order_id - 1]
        demand_by_color = {
            color: quantity
            for color, quantity in zip(("red", "blue", "green", "yellow", "white"), demand)
            if quantity
        }
        print(
            f"Order {order_id:02d}: requested {requested}, fulfillable {len(product_ids)}, "
            f"shortage {shortage}; color demand {demand_by_color}"
        )


if __name__ == "__main__":
    main()
