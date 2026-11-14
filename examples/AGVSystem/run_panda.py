"""Run the SOCS AGV model and display its live state in Panda3D."""
import argparse
import json
from math import cos, degrees, radians, sin
from pathlib import Path

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight, ClockObject, DirectionalLight, Geom, GeomNode, GeomTriangles,
    GeomVertexData, GeomVertexFormat, GeomVertexWriter, TextNode, Vec3,
    loadPrcFileData,
)
from socs.devs.TESEngin import TESEnginBase

from .agv_devs import make_model
from .traffic_map import TrafficMap
from .layout import BODY_OFFSETS, CONFIG, DWELL, FLOOR_DEPTH, FLOOR_WIDTH, SEGMENTS, VEHICLES, pose_behind


class LiveSimulation:
    """Advance DEVS only as far as the current display clock."""

    def __init__(self, model_factory=make_model):
        model, self.controller = model_factory()
        self.engine = TESEnginBase(model)
        self.engine.bVerbose = False
        self.time = 0.0
        self.read = 0
        self.departures = {}
        self.status = {v.name: f"Waiting at {v.start}" for v in VEHICLES}

    def advance(self, elapsed):
        self.time += elapsed
        while self.engine.model.tNext <= self.time:
            self.engine.execute_one_step()
            events = self.controller.state["events"]
            for kind, point, vehicle in events[self.read:]:
                if kind == "enter":
                    self.departures[vehicle] = (self.engine.tNow, point)
                    self.status[vehicle] = f"{point} -> {SEGMENTS[point].target}"
                elif kind == "arrive" and point in DWELL:
                    self.status[vehicle] = f"Charging at {point}"
                elif kind == "ready":
                    self.status[vehicle] = f"Waiting at {point}"
            self.read = len(events)

    def vehicle_pose(self, vehicle, offset):
        if vehicle.name not in self.departures:
            return pose_behind(vehicle.start, -offset)
        departure, source = self.departures[vehicle.name]
        distance = min((self.time - departure) * vehicle.speed, SEGMENTS[source].length)
        return pose_behind(source, distance - offset)


def add_mesh(parent, data):
    vertices = GeomVertexData(data["name"], GeomVertexFormat.get_v3n3(), Geom.UH_static)
    positions = GeomVertexWriter(vertices, "vertex")
    normals = GeomVertexWriter(vertices, "normal")
    triangles = GeomTriangles(Geom.UH_static)
    for triangle in data["triangles"]:
        a, b, c = (Vec3(*data["vertices"][i]) for i in triangle)
        normal = (b - a).cross(c - a)
        if not normal.normalize():
            continue
        start = positions.get_write_row()
        for vertex in (a, b, c):
            positions.add_data3(vertex)
            normals.add_data3(normal)
        triangles.add_vertices(start, start + 1, start + 2)
    geom = Geom(vertices)
    geom.add_primitive(triangles)
    node = GeomNode(data["name"])
    node.add_geom(geom)
    mesh = parent.attach_new_node(node)
    mesh.set_color(*data["color"])
    mesh.set_two_sided(True)


class AGVViewer(ShowBase):
    def __init__(self, speed=1.0, window_type=None, *, model_factory=make_model, panel_factory=TrafficMap):
        self.model_factory = model_factory
        super().__init__(windowType=window_type)
        self.disableMouse()
        self.setBackgroundColor(0.09, 0.13, 0.18)
        self.speed = speed
        self.paused = False
        self.azimuth = -90
        self.elevation = 55
        self.framing = max(FLOOR_WIDTH / 40, FLOOR_DEPTH / 26)
        self.distance = 48 * self.framing
        self.bodies = {
            name: self.render.attach_new_node(name)
            for parts in BODY_OFFSETS.values() for name, _ in parts
        }
        scene_path = Path(__file__).with_name("panda_scene.json")
        asset = json.loads(scene_path.read_text(encoding="utf-8"))
        if asset["dimensions"] != CONFIG:
            raise ValueError("Dimensions changed; run build_blender.py before launching Panda3D")
        for mesh in asset["geometry"]:
            add_mesh(self.bodies.get(mesh["body"], self.render), mesh)

        ambient = AmbientLight("Ambient")
        ambient.set_color((0.58, 0.58, 0.58, 1))
        self.render.set_light(self.render.attach_new_node(ambient))
        sun = DirectionalLight("Key")
        sun.set_color((0.63, 0.63, 0.63, 1))
        light = self.render.attach_new_node(sun)
        light.set_hpr(-35, -58, 0)
        self.render.set_light(light)
        self.traffic_map = panel_factory(self.aspect2d)
        self.split_view = True
        self.view_aspect = None
        self.camLens.set_fov(55)
        self.update_camera()

        self.hud = OnscreenText(
            parent=self.a2dTopLeft, pos=(0.05, -0.09), scale=0.048,
            align=TextNode.A_left, fg=(0.95, 0.97, 1, 1),
            shadow=(0, 0, 0, 1), mayChange=True,
        )
        OnscreenText(
            text="Tab  View   Space  Pause   R  Reset   Arrows  Orbit   Esc  Exit",
            parent=self.a2dBottomLeft, pos=(0.05, 0.05), scale=0.043,
            align=TextNode.A_left, fg=(0.95, 0.97, 1, 1), shadow=(0, 0, 0, 1),
        )
        self.accept("escape", self.userExit)
        self.accept("space", self.toggle_pause)
        self.accept("tab", self.toggle_view)
        self.accept("r", self.reset)
        for key, azimuth, elevation in (
            ("arrow_left", -12, 0), ("arrow_right", 12, 0),
            ("arrow_up", 0, 8), ("arrow_down", 0, -8),
        ):
            self.accept(key, self.orbit, [azimuth, elevation])
            self.accept(f"{key}-repeat", self.orbit, [azimuth, elevation])
        for key, distance in (("=", -4), ("-", 4), ("wheel_up", -4), ("wheel_down", 4)):
            self.accept(key, self.zoom, [distance])
        self.reset()
        self.taskMgr.add(self.tick, "live AGV simulation")

    def update_camera(self):
        azimuth, elevation = radians(self.azimuth), radians(self.elevation)
        horizontal = self.distance * cos(elevation)
        self.camera.set_pos(horizontal * cos(azimuth), horizontal * sin(azimuth),
                            self.distance * sin(elevation))
        self.camera.look_at(0, 0, 0)

    def orbit(self, azimuth, elevation):
        self.azimuth += azimuth
        self.elevation = max(15, min(85, self.elevation + elevation))
        self.update_camera()

    def zoom(self, distance):
        self.distance = max(12 * self.framing, min(85 * self.framing, self.distance + distance * self.framing))
        self.update_camera()

    def update_view(self):
        aspect = self.getAspectRatio()
        if self.view_aspect == aspect:
            return
        self.view_aspect = aspect
        right = 0.5 if self.split_view else 1.0
        self.camNode.get_display_region(0).set_dimensions(0, right, 0, 1)
        self.camLens.set_aspect_ratio(aspect * right)
        self.traffic_map.resize(aspect)

    def toggle_view(self):
        self.split_view = not self.split_view
        self.view_aspect = None
        if self.split_view:
            self.traffic_map.root.show()
        else:
            self.traffic_map.root.hide()
        self.draw_state()

    def toggle_pause(self):
        self.paused = not self.paused

    def reset(self):
        self.simulation = LiveSimulation(self.model_factory)
        self.simulation.advance(0)
        self.draw_state()

    def draw_state(self):
        self.update_view()
        simulation = self.simulation
        if self.split_view:
            self.traffic_map.update(simulation)
        for vehicle in VEHICLES:
            for name, offset in BODY_OFFSETS[vehicle.name]:
                x, y, heading = simulation.vehicle_pose(vehicle, offset)
                self.bodies[name].set_pos(x, y, 0)
                self.bodies[name].set_h(degrees(heading))
        mode = "Paused" if self.paused else f"{self.speed:g}x"
        rows = [f"Simulation  {simulation.time:06.2f} s  |  {mode}"]
        rows.extend(f"{name}: {status}" for name, status in simulation.status.items())
        self.hud.setText("\n".join(rows))

    def tick(self, task):
        if not self.paused:
            self.simulation.advance(ClockObject.get_global_clock().get_dt() * self.speed)
        self.draw_state()
        return task.cont


def main(model_factory=make_model, title="AGVSystem", panel_factory=TrafficMap):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()
    if not 0 < args.speed < float("inf"):
        parser.error("--speed must be positive and finite")
    loadPrcFileData("", f"window-title SOCS | {title}\nwin-size 1280 820\nsync-video true")
    AGVViewer(args.speed, model_factory=model_factory, panel_factory=panel_factory).run()


if __name__ == "__main__":
    main()
