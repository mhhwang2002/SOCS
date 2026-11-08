"""Animate an ACO-generated AS/RS schedule in the SOCS Panda3D viewer."""

from __future__ import annotations

import argparse
from threading import Lock

from socs.visual3D import VisualEntityState
from socs.visual3D.panda_viewer import Panda3DViewer

from .aco_optimization import AntColonyOptimizer, load_instance
from .asrs_devs import (
    AISLE_Y,
    ASRSController,
    BOX_COLORS,
    CELL_WIDTH,
    Crane,
    HOME_X,
    LOAD_CENTER_OFFSET,
    LOAD_HEIGHT,
    LEVEL_PITCH,
    RACK_COLUMNS,
    RACK_DEPTH,
    UNLOAD_STATION_XS,
    UNLOAD_Y,
    iter_cells,
    make_asrs_model,
)
from .run_asrs import ASRSSimEngineThread, add_rack_geometry


class ACOAnimationEngineThread(ASRSSimEngineThread):
    """Report completed orders while running the controller's ACO plan."""

    def __init__(self, model, *args, objective, penalty_weight, **kwargs):
        self.objective = objective
        self.penalty_weight = penalty_weight
        super().__init__(model, *args, **kwargs)
        self.animation_interval = self.TimeIntervalAnimation
        self._reported_orders = set()
        self._reported_summary = False
        self._hud_lock = Lock()
        self._show_statistics = False
        self._hud_snapshot = self._capture_hud_snapshot()

    def executeOneStep(self):
        result = super().executeOneStep()
        snapshot = self._capture_hud_snapshot()
        with self._hud_lock:
            self._hud_snapshot = snapshot
        for order_id, completion_time in sorted(snapshot["completion_times"].items()):
            if order_id not in self._reported_orders:
                penalty = snapshot["penalties"].get(order_id, 0)
                actual = snapshot["actual_counts"].get(order_id, 0)
                requested = snapshot["requested_counts"].get(order_id, 0)
                status = "COMPLETE" if actual == requested else f"PARTIAL (short {requested - actual})"
                print(
                    f"Order {order_id} leaves at simulation time "
                    f"{completion_time:.3f} - {status}; shipped "
                    f"{actual}/{requested}."
                )
                self._reported_orders.add(order_id)
        if snapshot["phase"] == "completed" and not self._reported_summary:
            completion_times = tuple(snapshot["completion_times"].values())
            if completion_times:
                print(
                    "Animated schedule makespan: "
                    f"{max(completion_times):.3f} simulation time units."
                )
                print(
                    "Animated mean order completion: "
                    f"{sum(completion_times) / len(completion_times):.3f}."
                )
            self._reported_summary = True
        return result

    def _capture_hud_snapshot(self):
        controller = self.controller
        delivered = controller.delivered_products_by_order
        actual_counts = {
            order_id: len(product_ids)
            for order_id, product_ids in delivered.items()
        }
        requested_counts = dict(controller.requested_order_items)
        complete_count = sum(
            actual_counts.get(order_id, 0) == requested
            for order_id, requested in requested_counts.items()
        )
        partial_count = sum(
            0 < actual_counts.get(order_id, 0) < requested
            for order_id, requested in requested_counts.items()
        )
        skipped_count = sum(
            actual_counts.get(order_id, 0) == 0
            for order_id in requested_counts
        ) if controller.phase == "completed" else 0
        return {
            "phase": self.controller.phase,
            "completion_times": dict(self.controller.completed_order_times),
            "penalties": dict(self.controller.order_penalties),
            "total_orders": len(requested_counts),
            "requested_counts": requested_counts,
            "actual_counts": actual_counts,
            "in_progress": {
                order_id: count
                for order_id, count in actual_counts.items()
                if count and order_id not in controller.completed_order_times
            },
            "assembling_orders": sum(
                bool(product_ids) and order_id not in controller.completed_order_times
                for order_id, product_ids in delivered.items()
            ),
            "complete_count": complete_count,
            "partial_count": partial_count,
            "skipped_count": skipped_count,
            "shortage_units": sum(
                max(0, requested_counts[order_id] - actual_counts.get(order_id, 0))
                for order_id in requested_counts
            ),
            "penalty_weight": self.penalty_weight,
            "time": self.tNow,
        }

    def toggle_statistics(self):
        with self._hud_lock:
            self._show_statistics = not self._show_statistics

    def toggle_animation(self):
        if self.TimeIntervalAnimation == float("inf"):
            self.TimeIntervalAnimation = self.animation_interval
            self._tNextAnimation = self.tNow + self.animation_interval
            print("Continuous animation enabled.")
        else:
            self.TimeIntervalAnimation = float("inf")
            self._tNextAnimation = float("inf")
            print("Continuous animation disabled; display updates at DEVS events.")

    def hud_text(self):
        with self._hud_lock:
            snapshot = {
                **self._hud_snapshot,
                "completion_times": dict(self._hud_snapshot["completion_times"]),
                "penalties": dict(self._hud_snapshot["penalties"]),
                "requested_counts": dict(self._hud_snapshot["requested_counts"]),
                "actual_counts": dict(self._hud_snapshot["actual_counts"]),
                "in_progress": dict(self._hud_snapshot["in_progress"]),
            }
            show_statistics = self._show_statistics

        if show_statistics:
            return self._format_statistics(snapshot)

        completed = snapshot["completion_times"]
        lines = [
            f"Objective: complete orders > lambda shortage > {self.objective}",
            f"Lambda: {snapshot['penalty_weight']:g}/item | weighted shortage: {snapshot['shortage_units'] * snapshot['penalty_weight']:g}",
            f"Animation: {'ON' if self.TimeIntervalAnimation != float('inf') else 'OFF'} (V toggles; OFF updates at DEVS events)",
            f"Orders departed: {len(completed)}/{snapshot['total_orders']}",
            f"Assembling: {snapshot['assembling_orders']} | Complete: {snapshot['complete_count']} | Partial: {snapshot['partial_count']}",
        ]
        if snapshot["in_progress"]:
            progress = " | ".join(
                f"O{order_id:02d} {count}/{snapshot['requested_counts'].get(order_id, count)}"
                for order_id, count in sorted(snapshot["in_progress"].items())[:3]
            )
            lines.append(f"Picking: {progress}")
        recent = sorted(completed.items(), key=lambda item: item[1])[-4:]
        for order_id, completion_time in recent:
            penalty = snapshot["penalties"].get(order_id, 0)
            shipped = snapshot["actual_counts"].get(order_id, 0)
            requested = snapshot["requested_counts"].get(order_id, 0)
            status = "COMPLETE" if shipped == requested else f"PARTIAL (short {requested - shipped})"
            lines.append(
                f"Order {order_id} leaves at simulation time "
                f"{completion_time:.3f} - {status} - "
                f"shipped {snapshot['actual_counts'].get(order_id, 0)}/"
                f"{snapshot['requested_counts'].get(order_id, 0)}"
            )
        if snapshot["phase"] == "completed":
            lines.append(f"Unprocessed orders: {snapshot['skipped_count']}")
            lines.append("Simulation complete. Press S for final statistics.")
        else:
            lines.append("Press S to show statistics.")
        return "\n".join(lines)

    @staticmethod
    def _format_statistics(snapshot):
        completion_times = snapshot["completion_times"]
        penalties = snapshot["penalties"]
        requested_counts = snapshot["requested_counts"]
        actual_counts = snapshot["actual_counts"]
        complete_ids = [
            order_id for order_id, requested in requested_counts.items()
            if actual_counts.get(order_id, 0) == requested
        ]
        partial_ids = [
            order_id for order_id, requested in requested_counts.items()
            if 0 < actual_counts.get(order_id, 0) < requested
        ]
        partial_times = [
            completion_times[order_id]
            for order_id in partial_ids if order_id in completion_times
        ]
        complete_times = [
            completion_times[order_id]
            for order_id in complete_ids if order_id in completion_times
        ]
        requested_total = sum(requested_counts.values())
        actual_total = sum(actual_counts.values())
        is_complete = snapshot["phase"] == "completed"
        makespan = max(completion_times.values(), default=None)

        def average(values):
            return f"{sum(values) / len(values):.3f}" if values else "N/A"

        title = "FINAL STATISTICS" if is_complete else "CURRENT STATISTICS (simulation running)"
        makespan_text = f"{makespan:.3f}" if makespan is not None else "N/A"
        return "\n".join(
            (
                title,
                f"Makespan: {makespan_text} simulation time units",
                f"Items shipped: {actual_total}/{requested_total} (actual/requested)",
                f"Complete orders: {len(complete_ids)}; average departure: {average(complete_times)}",
                f"Partial orders: {len(partial_ids)}; shipped {sum(actual_counts.get(i, 0) for i in partial_ids)}/{sum(requested_counts.get(i, 0) for i in partial_ids)}; average departure: {average(partial_times)}",
                f"Unshipped items: {snapshot['shortage_units']}; weighted penalty: {snapshot['shortage_units'] * snapshot['penalty_weight']:g}",
                f"Unprocessed orders: {snapshot['skipped_count']}",
                "Press S to return to live progress.",
            )
        )


def _visual_state(model):
    crane = next(child for child in model.children if isinstance(child, Crane))
    controller = next(child for child in model.children if isinstance(child, ASRSController))
    states = []
    for cell in iter_cells(model):
        color = BOX_COLORS.get(cell.box_color, (0.76, 0.82, 0.84, 1.0))
        states.append(
            VisualEntityState(
                cell.name,
                (
                    (cell.bay - 0.5) * CELL_WIDTH,
                    RACK_DEPTH / 2.0,
                    (cell.level - 1) * LEVEL_PITCH + LOAD_CENTER_OFFSET,
                ),
                color=color,
                visible=cell.occupied,
            )
        )

    box_slots = ((-0.39, -0.18), (0.0, -0.18), (0.39, -0.18), (-0.195, 0.20), (0.195, 0.20))
    for order_id, product_ids in controller.delivered_products_by_order.items():
        if order_id in controller.completed_order_times:
            if len(product_ids) < controller.requested_order_items.get(order_id, 0):
                lane_y = -3.72
            else:
                lane_y = -2.88
        else:
            lane_y = -2.02
        center_x = 0.75 + (order_id - 1) * 1.5
        for product_index, product_id in enumerate(product_ids):
            dx, dy = box_slots[product_index % len(box_slots)]
            states.append(
                VisualEntityState(
                    f"OrderProduct-{product_id}",
                    (center_x + dx, lane_y + dy, 0.17),
                    color=BOX_COLORS.get(controller.product_colors.get(product_id), (0.76, 0.82, 0.84, 1.0)),
                )
            )

    states.extend(
        (
            VisualEntityState("CraneChassis", (crane.x, AISLE_Y, 0.18), color=(0.95, 0.38, 0.08, 1.0)),
            VisualEntityState("CraneMast", (crane.x, AISLE_Y, 2.55), color=(0.95, 0.38, 0.08, 1.0)),
            VisualEntityState("CraneHoist", (crane.x, AISLE_Y, crane.z), color=(0.16, 0.20, 0.22, 1.0)),
            VisualEntityState(
                "ForkLeft",
                (crane.x - 0.18, crane.y - 0.5, crane.z + crane.fork_lift - 0.08),
                color=(0.88, 0.71, 0.22, 1.0),
            ),
            VisualEntityState(
                "ForkRight",
                (crane.x + 0.18, crane.y - 0.5, crane.z + crane.fork_lift - 0.08),
                color=(0.88, 0.71, 0.22, 1.0),
            ),
            VisualEntityState(
                "ForkCarriage",
                (crane.x, AISLE_Y + 0.05, crane.z + crane.fork_lift - 0.08),
                color=(0.88, 0.71, 0.22, 1.0),
            ),
            VisualEntityState(
                "ForkLoad",
                (crane.x, crane.y, crane.z + crane.fork_lift - 0.08 + LOAD_HEIGHT / 2.0),
                color=BOX_COLORS.get(crane.box_color, (0.70, 0.78, 0.80, 1.0)),
                visible=crane.has_load,
            ),
            VisualEntityState("CraneGuideLeft", (crane.x - 0.16, AISLE_Y, 2.55), color=(0.16, 0.20, 0.22, 1.0)),
            VisualEntityState("CraneGuideRight", (crane.x + 0.16, AISLE_Y, 2.55), color=(0.16, 0.20, 0.22, 1.0)),
            VisualEntityState("CraneWheelFL", (crane.x - 0.27, AISLE_Y - 0.31, 0.12), color=(0.12, 0.14, 0.16, 1.0)),
            VisualEntityState("CraneWheelFR", (crane.x - 0.27, AISLE_Y + 0.31, 0.12), color=(0.12, 0.14, 0.16, 1.0)),
            VisualEntityState("CraneWheelRL", (crane.x + 0.27, AISLE_Y - 0.31, 0.12), color=(0.12, 0.14, 0.16, 1.0)),
            VisualEntityState("CraneWheelRR", (crane.x + 0.27, AISLE_Y + 0.31, 0.12), color=(0.12, 0.14, 0.16, 1.0)),
        )
    )
    return states


def _add_order_staging(viewer, model, schedule):
    """Create collection trays and separate complete/partial order lanes."""
    controller = next(child for child in model.children if isinstance(child, ASRSController))
    length = RACK_COLUMNS * CELL_WIDTH + 3.0
    viewer.add_box(
        "Order staging floor",
        (RACK_COLUMNS * CELL_WIDTH / 2.0, -3.05, -0.06),
        (length, 2.30, 0.12),
        (0.34, 0.39, 0.43, 1.0),
    )
    lanes = (
        ("Collecting", -2.02, (0.36, 0.40, 0.43, 1.0)),
        ("Complete", -2.88, (0.12, 0.38, 0.25, 1.0)),
        ("Partial", -3.72, (0.65, 0.30, 0.08, 1.0)),
    )
    for lane_name, y, color in lanes:
        for order_id in controller.requested_order_items:
            x = 0.75 + (order_id - 1) * 1.5
            viewer.add_box(
                f"{lane_name} tray O{order_id:02d}",
                (x, y, 0.025),
                (1.35, 0.82, 0.05),
                color,
            )
    assigned_orders = set(schedule.product_assignments)
    for order_index, station in enumerate(schedule.order_station):
        order_id = order_index + 1
        x = 0.75 + (order_id - 1) * 1.5
        label = (
            ("L" if station == 0 else "R")
            if order_index in assigned_orders
            else "—"
        )
        viewer.add_world_text(
            label,
            (x, -2.02, 0.78),
            scale=0.48,
            color=(1.0, 0.91, 0.35, 1.0),
        )

    viewer.add_overlay_text(
        "order-staging-legend",
        "Order trays left-to-right: O01 to O10\nL/R: unloading station | Gray: collecting | Green: complete | Orange: partial",
        position=(-1.28, 0.50),
        scale=0.038,
    )


def _build_inventory_and_plan(instance, schedule):
    inventory = {}
    retrieval_plan = []
    for product in instance.products:
        inventory[(product.bay + 1, product.level + 1)] = {
            "box_id": product.product_id,
            "color": product.color,
        }
    for route_index, product_index in enumerate(schedule.product_order):
        product = instance.products[product_index]
        order_id = schedule.product_assignments[route_index]
        retrieval_plan.append(
            {
                "cell": f"Cell-B{product.bay + 1:02d}-L{product.level + 1}",
                "product_id": product.product_id,
                "color": product.color,
                "order_id": order_id + 1,
                "station": schedule.order_station[order_id],
            }
        )
    return inventory, retrieval_plan


def _add_second_station(viewer):
    station_x = UNLOAD_STATION_XS[1]
    viewer.add_box(
        "Right unload station deck",
        (station_x, UNLOAD_Y, 0.355),
        (0.95, 0.90, 0.12),
        (0.16, 0.28, 0.32, 1.0),
    )
    for dx in (-0.36, 0.36):
        for dy in (-0.32, 0.32):
            viewer.add_box(
                f"Right unload station leg {dx:+.2f} {dy:+.2f}",
                (station_x + dx, UNLOAD_Y + dy, 0.1475),
                (0.08, 0.08, 0.295),
                (0.10, 0.12, 0.14, 1.0),
            )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True, help="saved JSON problem instance")
    parser.add_argument("--seed", type=int, default=42, help="ACO search random seed")
    parser.add_argument("--objective", choices=("makespan", "mean"), default="makespan")
    parser.add_argument("--ants", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument(
        "--penalty-weight",
        type=float,
        default=50000.0,
        help="lambda cost per missing color-specific product (default: 50000)",
    )
    parser.add_argument("--speed", type=float, default=5.0, help="simulation-to-wall-time ratio")
    args = parser.parse_args(argv)

    instance = load_instance(args.instance)
    optimizer = AntColonyOptimizer(
        instance,
        args.objective,
        args.seed + (1000 if args.objective == "makespan" else 2000),
        ants=args.ants,
        iterations=args.iterations,
        penalty_weight=args.penalty_weight,
    )
    schedule, result, history = optimizer.run()
    print(f"ACO objective: maximize complete orders, then lambda-weighted shortage, then {args.objective}")
    print(f"Instance: {args.instance}")
    print(f"Penalty weight (simulation time / missing item): {args.penalty_weight}")
    print(f"ACO search seed: {args.seed}; makespan: {result.makespan:.3f}")
    print(f"Complete orders: {result.complete_order_count}/{len(result.requested_by_order)}")
    print(f"Unshipped items: {result.shortage_units}; weighted penalty: {result.shortage_units * args.penalty_weight:.3f}")
    print(f"Combined objective score: {optimizer.best_score:.3f}")
    print(f"Mean order completion time: {result.mean_order_completion_time:.3f}")
    print(f"Order priority: {[order_id + 1 for order_id in schedule.order_priority]}")
    print(f"ACO order-to-station assignment (L=0, R=1): {schedule.order_station}")
    print(f"Best objective score by iteration: {history}")

    inventory, retrieval_plan = _build_inventory_and_plan(instance, schedule)
    model = make_asrs_model(
        inventory=inventory,
        retrieval_plan=retrieval_plan,
        order_penalties={
            order_id + 1: penalty
            for order_id, penalty in enumerate(instance.order_penalties)
        },
        requested_order_items={
            order_id + 1: sum(instance.order_demands[order_id])
            for order_id, product_ids in enumerate(instance.order_products)
        },
    )
    engine = ACOAnimationEngineThread(
        model,
        _visual_state,
        objective=args.objective,
        penalty_weight=args.penalty_weight,
        TimeIntervalAnimation=0.05,
        TimeSimulationEnd=10000.0,
        real_time_factor=args.speed,
    )
    engine.StepByStep = False
    engine.bVerbose = False

    title = f"SOCS AS/RS ACO Demo - Complete orders + shortage + {args.objective}"
    viewer = Panda3DViewer(engine.state_buffer, title=title)
    add_rack_geometry(viewer)
    _add_second_station(viewer)
    _add_order_staging(viewer, model, schedule)
    for cell in iter_cells(model):
        viewer.create_box_entity(cell.name, (0.78, 0.76, LOAD_HEIGHT))
    for product_id in (task["product_id"] for task in retrieval_plan):
        viewer.create_box_entity(f"OrderProduct-{product_id}", (0.34, 0.34, 0.26))
    viewer.create_box_entity("CraneChassis", (0.82, 0.68, 0.24))
    viewer.create_box_entity("CraneMast", (0.16, 0.18, 5.1))
    viewer.create_box_entity("CraneHoist", (0.62, 0.50, 0.24))
    moving_boxes = {
        "CraneGuideLeft": (0.045, 0.045, 4.85),
        "CraneGuideRight": (0.045, 0.045, 4.85),
        "CraneWheelFL": (0.20, 0.12, 0.16),
        "CraneWheelFR": (0.20, 0.12, 0.16),
        "CraneWheelRL": (0.20, 0.12, 0.16),
        "CraneWheelRR": (0.20, 0.12, 0.16),
        "ForkLeft": (0.07, 1.0, 0.045),
        "ForkRight": (0.07, 1.0, 0.045),
        "ForkCarriage": (0.52, 0.42, 0.08),
    }
    for entity_id, size in moving_boxes.items():
        viewer.create_box_entity(entity_id, size)
    viewer.create_box_entity("ForkLoad", (0.78, 0.76, LOAD_HEIGHT))

    viewer.set_camera((8.0, -20.0, 13.0), target=(7.5, 0.0, 2.0))
    viewer.bind_key("space", engine.make_run)
    viewer.bind_key("a", lambda: _set_automatic(engine))
    viewer.bind_key("i", lambda: _set_interactive(engine))
    viewer.bind_key("v", engine.toggle_animation)
    viewer.bind_key("r", lambda: viewer.set_camera((8.0, -20.0, 13.0), target=(7.5, 0.0, 2.0)))
    viewer.add_overlay_text("simulation-status", engine.hud_text())
    viewer.bind_overlay_provider("simulation-status", engine.hud_text)
    viewer.bind_key("s", engine.toggle_statistics)
    viewer.bind_key("q", viewer.close)
    viewer.bind_key("escape", viewer.close)
    print(
        "Starting optimized dispatch: SPACE resume, A automatic, I step-by-step, "
        "V toggle continuous animation / DEVS-event display, "
        "S toggle statistics, R reset view, ESC/Q close."
    )

    engine.start()
    engine.make_run()
    try:
        viewer.run()
    finally:
        engine.terminate()


def _set_automatic(engine):
    engine.StepByStep = False
    engine.make_run()


def _set_interactive(engine):
    engine.StepByStep = True
    engine.make_run()


if __name__ == "__main__":
    main()
