"""Live 3D view of the Logistics Center experiment."""

import json
from pathlib import Path
from threading import RLock

from examples.ASRS.asrs_devs import AISLE_Y
from socs.visual3D import VisualEntityState, VisualTESEnginThread

from .cart_mesh import cart_meshes, tugger_meshes
from .model import (
    AGV_LANE_Y, BAY_ENTRIES, BAY_EXITS, CART_OFFSETS, FORK_OFFSETS, HORIZON, PARKING, PORTS,
    DEFAULT_LEAD_TIME, DEFAULT_REORDER_POINT, RACK_CAPACITY, RECEIVING, ROUTE, STATION,
    inventory_breakdown, make_model, route_leg,
)
from .motion import FORK_RETRACT_Y, agv_pose, crane_pose, handoff_progress, rounded

FLOW_COLORS = {
    "OUT": (1.0, 0.24, 0.16, 1),
    "RETURN": (0.17, 0.68, 1.0, 1),
    "IN": (1.0, 0.88, 0.20, 1),
}


CRANE_ORANGE = (0.95, 0.38, 0.08, 1)
CRANE_STEEL = (0.16, 0.20, 0.24, 1)
FORK_GOLD = (0.88, 0.71, 0.22, 1)
CRANE_PARTS = {
    "chassis": ((0.90, 0.70, 0.24), (0, 0, 0.22), CRANE_ORANGE),
    "mast": ((0.19, 0.23, 4.8), (0, 0, 2.55), CRANE_ORANGE),
    "cap": ((0.42, 0.36, 0.13), (0, 0, 4.94), CRANE_ORANGE),
    "guide_left": ((0.045, 0.055, 4.65), (-0.16, 0, 2.55), CRANE_STEEL),
    "guide_right": ((0.045, 0.055, 4.65), (0.16, 0, 2.55), CRANE_STEEL),
    "drive": ((0.28, 0.26, 0.18), (0.29, 0, 0.42), CRANE_STEEL),
    **{
        f"wheel_{x}_{y}": ((0.20, 0.12, 0.16), (x * 0.29, y * 0.31, 0.15), CRANE_STEEL)
        for x in (-1, 1) for y in (-1, 1)
    },
}


SKU_DISPLAY = {
    "A": ("O", (0.95, 0.51, 0.19, 1)),
    "C": ("P", (0.75, 0.44, 0.87, 1)),
    "B": ("G", (0.25, 0.8, 0.47, 1)),
}
RECEIVING_PAD = (RECEIVING[0], RECEIVING[1] + 1.55)
PICKING_PAD = (STATION[0], STATION[1] + 1.55)
INVENTORY_COLUMNS = (
    ("Rack", "rack", -0.98),
    ("Available", "available", -0.74),
    ("Allocated", "allocated", -0.44),
    ("New putaway", "putaway", -0.10),
)


def receiving_slots(state, now):
    waiting = {i for i, item in state["bins"].items() if item["where"] == "receiving"}
    for job in state["busy"].values():
        if job["action"] == "inbound" and handoff_progress(job, now, 1) < 1:
            waiting.update(job["bins"])
    return {
        i: (RECEIVING_PAD[0] + slot * 0.64, RECEIVING_PAD[1], 0.35)
        for slot, i in enumerate(sorted(waiting))
    }


class ViewerEngine(VisualTESEnginThread):
    def __init__(self, model):
        self.lock = RLock()
        super().__init__(model, snapshot, TimeIntervalAnimation=0.05,
                         TimeSimulationEnd=HORIZON, real_time_factor=1.0)
        self.bVerbose = False

    def execute_one_step(self):
        with self.lock:
            return super().execute_one_step()

    def restart(self):
        self.pause()
        with self.lock:
            self._last_animation_time = None
            self.reset()

    def change_speed(self, factor):
        self.real_time_factor = min(16.0, max(0.25, self.real_time_factor * factor))

    def status(self):
        with self.lock:
            controller = self.model.children[0]
            state = controller.state
            busy = state["busy"]
            mode = "Ready" if not self.tNow and self.SkipLoop else ("Paused" if self.SkipLoop else "Running")
            activity = " | ".join(f"{name}: {busy[name]['action'] if name in busy else 'idle'}"
                                  for name in ("crane", *controller.vehicles, "pick"))
            ports = " | ".join(f"{port}: {entry[0]} Bin {entry[1]}" if entry else f"{port}: free"
                               for port, entry in state["ports"].items())
            completed = sum(order["completed"] is not None for order in state["orders"].values())
            replenishment = " | ".join(
                f"{SKU_DISPLAY[sku][0]}: {'receiving' if eta is None else f'ETA {eta:g}s'}"
                for sku, eta in state["replenishment"].items()
            ) or "none"
            return (f"{mode}   t={self.tNow:.1f}s   {self.real_time_factor:g}x   flow={controller.flow}\n"
                    f"{activity}\n{ports}   Orders complete: {completed}/{len(state['orders'])}\n"
                    f"Pull <= {controller.reorder_point} / lead {controller.lead_time:g}s: {replenishment}\n"
                    "Space: Start/resume   P: Pause   R: Reset   U/D: Speed   Esc: Exit")


def snapshot(model):
    control = model.children[0]
    state = control.state
    now = model.tNow
    jobs = state["busy"]
    receiving = receiving_slots(state, now)
    crane = jobs.get("crane")
    crane_point, crane_stop, fork_y, lift, active_fork = (
        crane_pose(crane, now) if crane else ((7.5, 0), 0, FORK_RETRACT_Y, 0, None)
    )
    x, z = crane_point
    entities = [
        VisualEntityState(name, (x + dx, AISLE_Y + dy, height), color=color)
        for name, (_, (dx, dy, height), color) in CRANE_PARTS.items()
    ]
    entities.append(VisualEntityState(
        "hoist", (x, AISLE_Y, z + 0.20), color=CRANE_STEEL,
    ))
    for slot in range(control.forks):
        reach = fork_y if slot == active_fork else FORK_RETRACT_Y
        fork_lift = lift if slot == active_fork else 0
        fork_x = x + FORK_OFFSETS[slot]
        fork_z = z + 0.21 + fork_lift
        entities.append(VisualEntityState(
            f"carriage{slot}", (fork_x, reach, fork_z - 0.04), color=CRANE_STEEL,
        ))
        for side in (-1, 1):
            entities.append(VisualEntityState(
                f"fork{slot}_{side}", (fork_x + side * 0.12, reach, fork_z),
                color=FORK_GOLD,
            ))
    for name in control.vehicles:
        job = jobs.get(name)
        parked = not job or now < job["depart"]
        px, py = PARKING[name]
        point, heading, _ = ((px, py), -90, 0) if parked else agv_pose(job, now)
        entities.append(VisualEntityState(name, (*point, 0), heading=heading + 90))
        for trailer, offset in enumerate(CART_OFFSETS[:control.trailers]):
            point, heading, _ = ((px - offset, py), -90, 0) if parked else agv_pose(job, now, offset)
            entities.append(VisualEntityState(
                f"{name}_trailer{trailer}", (*point, 0), heading=heading + 90,
            ))
    for i, bin_data in state["bins"].items():
        where = bin_data["where"]
        x, z = bin_data["cell"]
        rack_point = (x + 0.5, 0.45, (z + 0.5) * 0.75)
        point = rack_point
        if where == "receiving":
            point = receiving[i]
        elif where in {"incoming", "out"}:
            queue = state["incoming"] if where == "incoming" else state["out"]
            offset = queue.index(i) if i in queue else 0
            point = (PICKING_PAD[0] + (-1 if where == "incoming" else 1),
                     PICKING_PAD[1] + offset * 0.6, 0.4)
        elif where in {"outbound", "port"}:
            port = next(p for p, entry in state["ports"].items() if entry and entry[1] == i)
            point = (*PORTS[port], 0.45)
        for job in jobs.values():
            if i not in job["bins"]:
                continue
            slot = job["bins"].index(i)
            if job["machine"] == "crane":
                x, z = crane_point
                segment = crane_stop
                first, second = 1 + slot, 1 + len(job["bins"]) + slot
                port = job["stops"][slot][0]
                if first <= segment < second:
                    reach = fork_y if slot == active_fork else FORK_RETRACT_Y
                    point = (x + FORK_OFFSETS[slot], reach, z + 0.375 + (lift if slot == active_fork else 0))
                else:
                    at_rack = (job["action"] == "store") == (segment >= second)
                    point = rack_point if at_rack else (*PORTS[port], 0.45)
            elif job["machine"].startswith("agv"):
                _, _, segment = agv_pose(job, now)
                port = job["stops"][slot][0]
                stop = job["port_indices"][slot]
                progress = handoff_progress(job, now, stop)
                trailer, _, _ = agv_pose(job, now, CART_OFFSETS[slot])
                port_point = (*PORTS[port], 0.45)
                cart_point = (*trailer, 0.65)
                if 0 < progress < 1:
                    fraction = progress if job["action"] == "deliver" else 1 - progress
                    point = tuple(a + (b - a) * fraction for a, b in zip(port_point, cart_point))
                elif job["action"] == "deliver" and progress == 0:
                    point = port_point
                elif job["action"] == "return" and segment < 1:
                    point = (PICKING_PAD[0] + 1, PICKING_PAD[1] + slot * 0.6, 0.4)
                elif job["action"] == "inbound" and i in receiving:
                    fraction = handoff_progress(job, now, 1)
                    point = tuple(a + (b - a) * fraction for a, b in zip(receiving[i], cart_point))
                elif job["action"] != "deliver" and progress == 1:
                    point = port_point
                elif job["action"] == "deliver" and segment >= job["drop_index"]:
                    point = (PICKING_PAD[0] - 1, PICKING_PAD[1] + slot * 0.6, 0.4)
                else:
                    point = cart_point
            else:
                point = (*PICKING_PAD, 0.55)
            break
        color = SKU_DISPLAY[bin_data["sku"]][1]
        entities.append(VisualEntityState(f"bin{i}", point, color=color,
                                          visible=where != "shipped"))
        mark = bin_data.get("flow_mark")
        entities.append(VisualEntityState(
            f"flow{i}", (point[0], point[1], point[2] + 0.25),
            color=FLOW_COLORS.get(mark), visible=mark is not None and where != "shipped",
        ))
        label_point = (point[0], point[1], point[2] + (0.38 if mark else 0.29))
        if point == rack_point:
            label_point = (point[0], -0.08, point[2] + 0.10)
        entities.append(VisualEntityState(
            f"count{i}", label_point,
            visible=where != "shipped", text=str(bin_data["stock"]),
        ))
    return entities


def run(forks, trailers, rule, flow="all",
        reorder_point=DEFAULT_REORDER_POINT, lead_time=DEFAULT_LEAD_TIME, agvs=2):
    from socs.visual3D.panda_viewer import Panda3DViewer
    from examples.ASRS.run_asrs import add_rack_geometry
    model, control = make_model(forks, trailers, rule, flow, reorder_point, lead_time, agvs)
    from direct.gui.DirectGui import DirectButton
    from examples.AGVSystem.run_panda import add_mesh

    engine = ViewerEngine(model)
    viewer = Panda3DViewer(engine.state_buffer, title="Logistics Center")
    add_rack_geometry(viewer, show_shelf_decks=False)
    viewer.add_box("AGV floor", (7, -6.5, -0.07), (35, 14, 0.12), (0.26, 0.29, 0.33, 1))
    for name, (x, y) in {**PORTS, "Receiving": RECEIVING_PAD, "Picking": PICKING_PAD}.items():
        viewer.add_box(name, (x, y, 0.10), (0.8, 0.8, 0.20), (0.88, 0.73, 0.2, 1))
        if name not in PORTS:
            viewer.add_world_text(name, (x, y, 0.75), scale=0.28)
    for name, (x, y) in PORTS.items():
        edge = AGV_LANE_Y + 0.60
        viewer.add_box(name + " transfer", (x, (y + edge) / 2, 0.19),
                       (0.54, y - edge, 0.10), (0.28, 0.47, 0.53, 1))
    loop = [ROUTE[1], *ROUTE[2:], ROUTE[0], ROUTE[1]]
    viewer.add_polyline([(x, y, 0.025) for x, y in rounded(loop)],
                        color=(0.80, 0.84, 0.87, 1), thickness=2.0)
    for name in control.vehicles:
        bay = PARKING[name]
        lane = route_leg(BAY_ENTRIES[name], bay) + route_leg(bay, BAY_EXITS[name])[1:]
        viewer.add_polyline([(x, y, 0.025) for x, y in rounded(lane)], thickness=2.0)
    for name, (size, _, _) in CRANE_PARTS.items():
        viewer.create_box_entity(name, size)
    viewer.create_box_entity("hoist", (1.43 if forks == 2 else 1.15, 0.42, 0.22))
    for slot in range(forks):
        viewer.create_box_entity(f"carriage{slot}", (0.36, 0.42, 0.09))
        for side in (-1, 1):
            viewer.create_box_entity(f"fork{slot}_{side}", (0.07, 0.98, 0.045))

    scene = Path(__file__).resolve().parents[1] / "AGVSystem" / "panda_scene.json"
    meshes = json.loads(scene.read_text(encoding="utf-8"))["geometry"]

    def attach_vehicle(name, body):
        root = viewer._base.render.attach_new_node(name)
        viewer.attach_entity(name, root)
        if body == "Tugger":
            for mesh in tugger_meshes():
                add_mesh(root, mesh)
        else:
            for mesh in meshes:
                if mesh["body"] == body and mesh["name"].startswith("Drawbar"):
                    add_mesh(root, mesh)
            for mesh in cart_meshes():
                add_mesh(root, mesh)

    for name in control.vehicles:
        attach_vehicle(name, "Tugger")
        for trailer in range(trailers):
            attach_vehicle(f"{name}_trailer{trailer}", "Cart 1" if trailer == 0 else "Cart 2")
    for i in range(RACK_CAPACITY):
        viewer.create_box_entity(f"bin{i}", (0.43, 0.43, 0.37))
        marker = viewer.create_box_entity(f"flow{i}", (0.38, 0.38, 0.09))
        marker.hide()
        count_label = viewer.add_world_text("", (0, 0, 0), scale=0.23)
        viewer.attach_entity(f"count{i}", count_label)
        count_label.hide()
    viewer.set_camera((23, -32, 22), target=(7.5, -5.5, 1.4))
    viewer.add_overlay_text("status", engine.status(), position=(-1.25, 0.92), scale=0.038)
    viewer.bind_overlay_provider("status", engine.status)
    viewer.add_overlay_text("stock-title", "SKU", position=(-1.25, 0.68), scale=0.034)
    for title, key, x in INVENTORY_COLUMNS:
        viewer.add_overlay_text(f"stock-title-{key}", title, position=(x, 0.68),
                                scale=0.034, align="center")
    for row, (sku, (label, color)) in enumerate(SKU_DISPLAY.items()):
        y = 0.635 - row * 0.045
        viewer.add_overlay_text(f"stock-{sku}", label, position=(-1.25, y),
                                scale=0.034, color=color)
        for _, key, x in INVENTORY_COLUMNS:
            viewer.add_overlay_text(f"stock-{sku}-{key}", "", position=(x, y),
                                    scale=0.034, align="center")

    def update_inventory():
        with engine.lock:
            inventory = inventory_breakdown(engine.model.children[0].state)
        for sku, item in inventory.items():
            values = {**item, "allocated": f"{item['reserved']} + {item['picked']}"}
            for _, key, _ in INVENTORY_COLUMNS:
                viewer.set_overlay_text(f"stock-{sku}-{key}", str(values[key]))
        return ""

    viewer.add_overlay_text("stock-refresh", "")
    viewer.bind_overlay_provider("stock-refresh", update_inventory)
    for index, (label, color) in enumerate(FLOW_COLORS.items()):
        viewer.add_overlay_text("legend-" + label, label,
                                position=(-1.25 + index * 0.26, 0.475), scale=0.04, color=color)
    buttons = [
        DirectButton(text=label, scale=0.045, pos=(x, 0, -0.88), command=command)
        for label, x, command in (("Start", -1.10, engine.make_run), ("Reset", -0.87, engine.restart))
    ]
    viewer.bind_key("space", engine.make_run)
    viewer.bind_key("p", engine.pause)
    viewer.bind_key("r", engine.restart)
    viewer.bind_key("u", lambda: engine.change_speed(2))
    viewer.bind_key("d", lambda: engine.change_speed(0.5))
    viewer.bind_key("escape", viewer.close)
    engine.start()
    try:
        viewer.run()
    finally:
        engine.terminate()
        for button in buttons:
            button.destroy()


