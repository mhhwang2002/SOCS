"""Generate fixed AS/RS instances with controlled adjacent-cell pairability."""

from __future__ import annotations

import argparse
import json
import random
from collections import defaultdict
from functools import lru_cache
from pathlib import Path

from examples.ASRS.aco_optimization import ASRSInstance, Product, load_instance, save_instance


SCENARIOS = {
    "almost-none": 4,
    "low": 8,
    "medium": 20,
    "high": 40,
}
RACK_COLUMNS = 15
RACK_LEVELS = 6
PRODUCT_COUNT = 50


def count_adjacent_pairs(products):
    occupied = {(product.bay, product.level) for product in products}
    return sum(
        (bay + 1, level) in occupied
        for bay, level in occupied
    )


def count_order_compatible_pairs(instance):
    products_by_cell = {
        (product.bay, product.level): product
        for product in instance.products
    }
    color_order = {color: index for index, color in enumerate(("red", "blue", "green", "yellow", "white"))}
    compatible = 0
    for (bay, level), first in products_by_cell.items():
        second = products_by_cell.get((bay + 1, level))
        if second is None:
            continue
        first_color = color_order[first.color]
        second_color = color_order[second.color]
        if any(
            demand[first_color] >= 1
            and demand[second_color] >= (2 if first_color == second_color else 1)
            for demand in instance.order_demands
        ):
            compatible += 1
    return compatible


def _row_patterns():
    patterns = defaultdict(list)
    for mask in range(1 << RACK_COLUMNS):
        count = mask.bit_count()
        adjacent = sum(
            bool(mask & (1 << bay)) and bool(mask & (1 << (bay + 1)))
            for bay in range(RACK_COLUMNS - 1)
        )
        patterns[(count, adjacent)].append(mask)
    return patterns


def make_occupancy(target_pairs, rng):
    patterns = _row_patterns()
    stats = tuple(patterns)

    @lru_cache(maxsize=None)
    def can_complete(level, remaining_products, remaining_pairs):
        if level == RACK_LEVELS:
            return remaining_products == 0 and remaining_pairs == 0
        return any(
            count <= remaining_products
            and adjacent <= remaining_pairs
            and can_complete(level + 1, remaining_products - count, remaining_pairs - adjacent)
            for count, adjacent in stats
        )

    if not can_complete(0, PRODUCT_COUNT, target_pairs):
        raise ValueError(f"Cannot place {PRODUCT_COUNT} products with {target_pairs} adjacent pairs.")

    cells = []
    remaining_products = PRODUCT_COUNT
    remaining_pairs = target_pairs
    for level in range(RACK_LEVELS):
        options = [
            (count, adjacent)
            for count, adjacent in stats
            if count <= remaining_products
            and adjacent <= remaining_pairs
            and can_complete(level + 1, remaining_products - count, remaining_pairs - adjacent)
        ]
        count, adjacent = rng.choice(options)
        mask = rng.choice(patterns[(count, adjacent)])
        cells.extend(
            (bay, level)
            for bay in range(RACK_COLUMNS)
            if mask & (1 << bay)
        )
        remaining_products -= count
        remaining_pairs -= adjacent
    return cells


def make_instance(base, scenario, target_pairs, seed):
    rng = random.Random(seed)
    cells = make_occupancy(target_pairs, rng)
    rng.shuffle(cells)
    products = tuple(
        Product(
            product.product_id,
            product.color,
            cell[0],
            cell[1],
            product.order_id,
        )
        for product, cell in zip(base.products, cells)
    )
    return ASRSInstance(
        products,
        base.order_products,
        base.order_penalties,
        base.order_demands,
    )


def generate_instances(base_path, output_dir, seed=42):
    base_path = Path(base_path)
    output_dir = Path(output_dir)
    base = load_instance(base_path)
    paths = []
    for index, (scenario, target_pairs) in enumerate(SCENARIOS.items()):
        instance = make_instance(base, scenario, target_pairs, seed + index)
        actual_pairs = count_adjacent_pairs(instance.products)
        if actual_pairs != target_pairs:
            raise RuntimeError(f"{scenario}: requested {target_pairs} adjacent pairs, got {actual_pairs}.")
        path = save_instance(instance, output_dir / f"pairability_{scenario}.json", seed + index)
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["layout_scenario"] = {
            "name": scenario,
            "adjacent_occupied_pairs": actual_pairs,
            "order_compatible_adjacent_pairs": count_order_compatible_pairs(instance),
            "base_instance": str(base_path),
            "layout_seed": seed + index,
            "products_and_order_demands_preserved": True,
        }
        path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        paths.append(path)
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-instance", default="examples/ASRS/instances/demo.json")
    parser.add_argument(
        "--output-dir",
        default="examples/ASRS_2ForkCrane/instances/pairability",
    )
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args(argv)
    for path in generate_instances(args.base_instance, args.output_dir, args.seed):
        payload = json.loads(path.read_text(encoding="utf-8"))
        scenario = payload["layout_scenario"]
        print(
            f"{path}: adjacent={scenario['adjacent_occupied_pairs']}, "
            f"order-compatible={scenario['order_compatible_adjacent_pairs']}"
        )


if __name__ == "__main__":
    main()
