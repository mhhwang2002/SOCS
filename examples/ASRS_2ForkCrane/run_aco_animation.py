"""Run and animate an ACO schedule on the two-fork AS/RS DEVS model."""

from __future__ import annotations

import argparse
from threading import Lock

from socs.devs.TESEngin import ExecutionResult, TESEnginBase
from socs.visual3D import VisualEntityState, VisualTESEnginThread
from socs.visual3D.panda_viewer import Panda3DViewer

from examples.ASRS.run_asrs import add_rack_geometry, reset_camera
from .aco_optimization import (
    TwoForkAntColonyOptimizer,
    build_retrieval_plan,
    load_instance,
)
from .asrs_devs import (
    AISLE_Y,
    ASRSController,
    BOX_COLORS,
    CELL_WIDTH,
    HOME_Z,
    LEVEL_PITCH,
    LOAD_HEIGHT,
    RACK_COLUMNS,
    RACK_DEPTH,
    UNLOAD_STATION_XS,
    UNLOAD_Y,
    TwoForkCrane,
    iter_cells,
    make_asrs_model,
)
from .timing import DEFAULT_TIMING_PROFILE, TimingProfile


class TwoForkAnimationEngineThread(VisualTESEnginThread):
    """Publish every DEVS state and stop after the controller empties its plan."""

    def __init__(self, model, *args, objective, penalty_weight, picking_mode, **kwargs):
        self.controller = next(
            child for child in model.children if isinstance(child, ASRSController)
        )
        self.objective = objective
        self.penalty_weight = penalty_weight
        self.picking_mode = picking_mode
        self._hud_lock = Lock()
        self._show_statistics = False
        self._reported_orders = set()
        super().__init__(model, *args, **kwargs)
        self._hud_snapshot = self._capture_hud()

    def execute_one_step(self):
        result = super().execute_one_step()
        snapshot = self._capture_hud()
        with self._hud_lock:
            self._hud_snapshot = snapshot
        for order_id, time in sorted(snapshot["completion_times"].items()):
            if order_id not in self._reported_orders:
                shipped = snapshot["shipped"].get(order_id, 0)
                requested = snapshot["requested"].get(order_id, 0)
                status = "COMPLETE" if shipped == requested else f"PARTIAL, shipped {shipped}/{requested}"
                print(f"Order {order_id} leaves at simulation time {time:.3f} - {status}.")
                self._reported_orders.add(order_id)
        if snapshot["phase"] == "completed" and not self.SkipLoop:
            print(f"Two-fork DEVS simulation complete at {self.tNow:.3f}.")
            self.pause()
        return result

    def _capture_hud(self):
        delivered = self.controller.delivered_products_by_order
        shipped = {order_id: len(products) for order_id, products in delivered.items()}
        requested = dict(self.controller.requested_order_items)
        completed = dict(self.controller.completed_order_times)
        complete_count = sum(shipped.get(order_id, 0) >= count for order_id, count in requested.items())
        partial_count = sum(0 < shipped.get(order_id, 0) < count for order_id, count in requested.items())
        return {
            "phase": self.controller.phase,
            "time": self.tNow,
            "completion_times": completed,
            "requested": requested,
            "shipped": shipped,
            "complete_count": complete_count,
            "partial_count": partial_count,
            "shortage": sum(max(0, count - shipped.get(order_id, 0)) for order_id, count in requested.items()),
            "delivered": {order_id: list(products) for order_id, products in delivered.items()},
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
            snapshot = dict(self._hud_snapshot)
            show_statistics = self._show_statistics
        mode = "ON" if self.TimeIntervalAnimation != float("inf") else "DEVS events only"
        if show_statistics:
            return "\n".join(
                (
                    "TWO-FORK FINAL / CURRENT STATISTICS",
                    f"Simulation time: {snapshot['time']:.3f}",
                    f"Orders complete: {snapshot['complete_count']}",
                    f"Orders partial: {snapshot['partial_count']}",
                    f"Items shipped: {sum(snapshot['shipped'].values())}/{sum(snapshot['requested'].values())}",
                    f"Shortage penalty: {snapshot['shortage'] * self.penalty_weight:g}",
                    f"Picking mode: {self.picking_mode}",
                    "Press S to return to live progress.",
                )
            )
        return "\n".join(
            (
                f"Two-fork ACO objective: complete orders > lambda shortage > {self.objective}",
                f"Picking mode: {self.picking_mode}",
                f"Animation: {mode} (V toggles)",
                f"Orders complete: {snapshot['complete_count']} | partial: {snapshot['partial_count']}",
                f"Items shipped: {sum(snapshot['shipped'].values())}/{sum(snapshot['requested'].values())}",
                f"Simulation time: {snapshot['time']:.3f}",
                "Press S for statistics.",
            )
        )


def _visual_state(model, product_colors):
    crane = next(child for child in model.children if isinstance(child, TwoForkCrane))
    controller = next(child for child in model.children if isinstance(child, ASRSController))
    states = []
    for cell in iter_cells(model):
        states.append(
            VisualEntityState(
                cell.name,
                (
                    (cell.bay - 0.5) * CELL_WIDTH,
                    RACK_DEPTH / 2.0,
                    (cell.level - 1) * LEVEL_PITCH + 0.27,
                ),
                color=BOX_COLORS.get(cell.box_color, (0.76, 0.82, 0.84, 1.0)),
                visible=cell.occupied,
            )
        )

    for order_id, product_ids in controller.delivered_products_by_order.items():
        requested = controller.requested_order_items.get(order_id, 0)
        if order_id in controller.completed_order_times:
            lane_y = -2.88 if len(product_ids) >= requested else -3.72
        else:
            lane_y = -2.02
        center_x = 0.75 + (order_id - 1) * 1.5
        box_slots = ((-0.39, -0.18), (0.0, -0.18), (0.39, -0.18), (-0.195, 0.20), (0.195, 0.20))
        for index, product_id in enumerate(product_ids):
            dx, dy = box_slots[index % len(box_slots)]
            states.append(
                VisualEntityState(
                    f"OrderProduct-{product_id}",
                    (center_x + dx, lane_y + dy, 0.17),
                    color=BOX_COLORS.get(product_colors.get(product_id), (0.76, 0.82, 0.84, 1.0)),
                )
            )

    states.extend(
        (
            VisualEntityState("CraneChassis", (crane.x, AISLE_Y, 0.18), color=(0.95, 0.38, 0.08, 1.0)),
            VisualEntityState("CraneMast", (crane.x, AISLE_Y, 2.55), color=(0.95, 0.38, 0.08, 1.0)),
            VisualEntityState("CraneHoist", (crane.x, AISLE_Y, crane.z), color=(0.16, 0.20, 0.22, 1.0)),
            VisualEntityState("Fork1", (crane.x + crane.fork1.x_offset, AISLE_Y + crane.fork1.reach / 2.0, crane.z + crane.fork1.lift - 0.08), color=(0.88, 0.71, 0.22, 1.0)),
            VisualEntityState("Fork2", (crane.x + crane.fork2.x_offset, AISLE_Y + crane.fork2.reach / 2.0, crane.z + crane.fork2.lift - 0.08), color=(0.88, 0.71, 0.22, 1.0)),
            VisualEntityState("ForkLoad1", (crane.x + crane.fork1.x_offset, AISLE_Y + crane.fork1.reach, crane.z + crane.fork1.lift - 0.08 + LOAD_HEIGHT / 2.0), color=BOX_COLORS.get(crane.fork1.box_color, (0.70, 0.78, 0.80, 1.0)), visible=crane.fork1.has_load),
            VisualEntityState("ForkLoad2", (crane.x + crane.fork2.x_offset, AISLE_Y + crane.fork2.reach, crane.z + crane.fork2.lift - 0.08 + LOAD_HEIGHT / 2.0), color=BOX_COLORS.get(crane.fork2.box_color, (0.70, 0.78, 0.80, 1.0)), visible=crane.fork2.has_load),
        )
    )
    return states


def _add_order_staging(viewer, order_count):
    length = RACK_COLUMNS * CELL_WIDTH + 3.0
    viewer.add_box("Order staging floor", (RACK_COLUMNS * CELL_WIDTH / 2.0, -3.05, -0.06), (length, 2.30, 0.12), (0.34, 0.39, 0.43, 1.0))
    lanes = (
        ("Collecting", -2.02, (0.36, 0.40, 0.43, 1.0)),
        ("Complete", -2.88, (0.12, 0.38, 0.25, 1.0)),
        ("Partial", -3.72, (0.65, 0.30, 0.08, 1.0)),
    )
    for lane_name, y, color in lanes:
        for order_id in range(1, order_count + 1):
            x = 0.75 + (order_id - 1) * 1.5
            viewer.add_box(f"{lane_name} tray O{order_id:02d}", (x, y, 0.025), (1.35, 0.82, 0.05), color)


def _add_right_station(viewer):
    station_x = UNLOAD_STATION_XS[1]
    viewer.add_box("Right unload station deck", (station_x, UNLOAD_Y, 0.355), (0.95, 0.90, 0.12), (0.16, 0.28, 0.32, 1.0))
    for dx in (-0.36, 0.36):
        for dy in (-0.32, 0.32):
            viewer.add_box(f"Right station leg {dx:+.2f} {dy:+.2f}", (station_x + dx, UNLOAD_Y + dy, 0.1475), (0.08, 0.08, 0.295), (0.10, 0.12, 0.14, 1.0))


def run_headless(model):
    """Run a complete coupled retrieval plan without opening the 3D viewer."""
    controller = next(child for child in model.children if isinstance(child, ASRSController))
    engine = TESEnginBase(
        model,
        TimeIntervalContinuous=float("inf"),
        TimeIntervalAnimation=float("inf"),
        TimeSimulationEnd=1e9,
    )
    engine.bVerbose = False
    while controller.phase != "completed":
        result = engine.execute_one_step()
        if result is ExecutionResult.TimeBoundReached:
            raise RuntimeError("Two-fork coupled model did not complete before its time bound.")
    shipped = sum(len(products) for products in controller.delivered_products_by_order.values())
    return engine.tNow, shipped, controller


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True, help="saved JSON problem instance")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--objective", choices=("makespan", "mean"), default="makespan")
    parser.add_argument("--ants", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--penalty-weight", type=float, default=50000.0)
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument(
        "--station-policy",
        choices=("first-pickup", "whole-order"),
        default="first-pickup",
        help="choose L/R on the first pickup or after evaluating the whole order route",
    )
    parser.add_argument(
        "--picking-mode",
        choices=("single", "double", "sequential", "all"),
        default="double",
        help="single: one pick; double: force simultaneous pairs when feasible, else single; sequential: force sequential pairs when feasible; all: strict simultaneous > sequential > single priority",
    )
    parser.add_argument(
        "--timing-profile",
        default=str(DEFAULT_TIMING_PROFILE),
        help="DEVS-calibrated movement and fork-cycle time JSON",
    )
    parser.add_argument("--headless", action="store_true", help="run the coupled DEVS model without Panda3D")
    args = parser.parse_args(argv)

    instance = load_instance(args.instance)
    optimizer = TwoForkAntColonyOptimizer(
        instance,
        args.objective,
        args.seed,
        ants=args.ants,
        iterations=args.iterations,
        penalty_weight=args.penalty_weight,
        timing_profile=TimingProfile.load(args.timing_profile),
        station_policy=args.station_policy,
        picking_mode=args.picking_mode,
    )
    schedule, result, history = optimizer.run()
    plan = build_retrieval_plan(instance, schedule)
    inventory = {
        (product.bay + 1, product.level + 1): {
            "box_id": product.product_id,
            "color": product.color,
        }
        for product in instance.products
    }
    requested = {
        order_id + 1: sum(instance.order_demands[order_id])
        for order_id in range(len(instance.order_products))
    }
    model = make_asrs_model(
        inventory=inventory,
        retrieval_plan=plan,
        requested_order_items=requested,
    )
    mode_counts = {
        mode: schedule.pickup_modes.count(mode)
        for mode in ("single", "simultaneous", "sequential")
    }
    print(f"ACO objective score: {optimizer.best_score:.3f}")
    print(f"Unloading-station policy: {optimizer.station_policy}")
    print(f"Picking mode: {optimizer.picking_mode}")
    print(f"ACO estimate: makespan {result.makespan:.3f}; mean departure {result.mean_order_completion_time:.3f}")
    print(
        f"ACO estimate: {result.complete_order_count} complete; shortage {result.shortage_units}; "
        f"trips single/simultaneous/sequential={mode_counts['single']}/"
        f"{mode_counts['simultaneous']}/{mode_counts['sequential']}"
    )

    if args.headless:
        elapsed, shipped, controller = run_headless(model)
        completed = len(controller.completed_order_times)
        print(f"Coupled two-fork DEVS run: {elapsed:.3f} simulation time units")
        print(f"Shipped: {shipped}/{sum(requested.values())}; departed: {completed}/{len(requested)}")
        return

    product_colors = {product.product_id: product.color for product in instance.products}
    state_provider = lambda current_model: _visual_state(current_model, product_colors)
    engine = TwoForkAnimationEngineThread(
        model,
        state_provider,
        objective=args.objective,
        penalty_weight=args.penalty_weight,
        picking_mode=args.picking_mode,
        TimeIntervalAnimation=0.05,
        TimeSimulationEnd=10000.0,
        real_time_factor=args.speed,
    )
    engine.StepByStep = False
    engine.bVerbose = False

    viewer = Panda3DViewer(engine.state_buffer, title="SOCS AS/RS Two-Fork Crane Demo")
    add_rack_geometry(viewer)
    _add_right_station(viewer)
    _add_order_staging(viewer, len(instance.order_products))
    for cell in iter_cells(model):
        viewer.create_box_entity(cell.name, (0.78, 0.76, LOAD_HEIGHT))
    for product_id in (product.product_id for product in instance.products):
        viewer.create_box_entity(f"OrderProduct-{product_id}", (0.34, 0.34, 0.26))
    for entity_id, size in {
        "CraneChassis": (0.82, 0.68, 0.24),
        "CraneMast": (0.16, 0.18, 5.1),
        "CraneHoist": (0.62, 0.50, 0.24),
        "Fork1": (0.08, 1.2, 0.05),
        "Fork2": (0.08, 1.2, 0.05),
        "ForkLoad1": (0.72, 0.68, LOAD_HEIGHT),
        "ForkLoad2": (0.72, 0.68, LOAD_HEIGHT),
    }.items():
        viewer.create_box_entity(entity_id, size)

    reset_camera(viewer)
    viewer.bind_key("space", engine.make_run)
    viewer.bind_key("a", lambda: _run_automatically(engine))
    viewer.bind_key("i", lambda: _run_interactively(engine))
    viewer.bind_key("v", engine.toggle_animation)
    viewer.bind_key("s", engine.toggle_statistics)
    viewer.bind_key("r", lambda: reset_camera(viewer))
    viewer.bind_key("escape", viewer.close)
    viewer.bind_key("q", viewer.close)
    viewer.add_overlay_text("simulation-status", engine.hud_text())
    viewer.bind_overlay_provider("simulation-status", engine.hud_text)
    print("Two-fork AS/RS: SPACE resume, A automatic, I interactive, V event-only display, S statistics, R camera reset, Esc/Q close.")

    engine.start()
    engine.make_run()
    try:
        viewer.run()
    finally:
        engine.terminate()


def _run_automatically(engine):
    engine.StepByStep = False
    engine.make_run()


def _run_interactively(engine):
    engine.StepByStep = True
    engine.make_run()


if __name__ == "__main__":
    main()
