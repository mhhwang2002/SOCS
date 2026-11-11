"""Build an editable Blender scene and animate a replay of SOCS events."""
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from examples.AGVSystem.agv_devs import simulate
from examples.AGVSystem.layout import (
    BODY_OFFSETS, CHARGER, CONFIG, DWELL, FLOOR_DEPTH, FLOOR_WIDTH, LANE_WIDTH,
    POINTS, REFERENCE_OFFSETS, REFERENCE_SIZE, SEGMENTS, VEHICLES, pose_behind, vehicle_scale,
)
from examples.AGVSystem.export_panda import main as export_panda

SECONDS = 90
FPS = 24
PALETTE = {
    "floor": (0.57, 0.64, 0.68), "road": (0.13, 0.18, 0.24),
    "line": (0.88, 0.92, 0.95), "amber": (1.0, 0.58, 0.09),
    "green": (0.12, 0.73, 0.42), "ink": (0.05, 0.09, 0.14),
    "blue": (0.12, 0.62, 0.95), "red": (0.96, 0.32, 0.18),
}


def style(obj, name, color):
    obj.name = name
    obj.color = (*PALETTE[color], 1)
    return obj


def box(name, size, position, color, parent=None):
    bpy.ops.mesh.primitive_cube_add(size=1)
    obj = style(bpy.context.object, name, color)
    obj.parent = parent
    obj.location = position
    obj.dimensions = size
    return obj


def label(name, x, y, size=0.42, color="ink", z=0.055):
    font = bpy.data.curves.new(name, "FONT")
    font.body = name
    font.size = size
    font.align_x = "CENTER"
    obj = style(bpy.data.objects.new(name, font), name, color)
    bpy.context.collection.objects.link(obj)
    obj.location = x, y, z
    return obj


def strip(segment, offset, width, z, color, name):
    verts = []
    steps = 48 if segment.center else 1
    for i in range(steps + 1):
        x, y, a = segment.pose(i / steps)
        for d in (offset - width / 2, offset + width / 2):
            verts.append((x - math.sin(a) * d, y + math.cos(a) * d, z))
    faces = [(2*i, 2*i+2, 2*i+3, 2*i+1) for i in range(steps)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    obj = style(bpy.data.objects.new(name, mesh), name, color)
    bpy.context.collection.objects.link(obj)
    return obj


def build_track():
    box("Factory floor", (FLOOR_WIDTH, FLOOR_DEPTH, 0.16), (0, 0, -0.08), "floor")
    for source, segment in SEGMENTS.items():
        road = strip(segment, 0, LANE_WIDTH, 0.015, "road", f"SEG_{source}_{segment.target}")
        road["length_m"] = segment.length
        for side in (-1, 1):
            strip(segment, side * (LANE_WIDTH / 2 - 0.07), 0.035, 0.022, "line", f"Edge {source}")
        x, y, angle = segment.pose(0.50)
        arrow = label("▶", x, y, 0.45, "line")
        arrow.rotation_euler.z = angle
    for name, (x, y) in POINTS.items():
        angle = SEGMENTS[name].pose(0)[2]
        color = "green" if name in DWELL else "amber"
        marker = box(name, (0.095, LANE_WIDTH * 0.875, 0.028), (x, y, 0.03), color)
        marker.rotation_euler.z = angle
        marker["dwell_s"] = DWELL.get(name, 0)
        label(name, x - math.sin(angle) * (LANE_WIDTH / 2 + 0.5), y + math.cos(angle) * (LANE_WIDTH / 2 + 0.5) - 0.13)
        if name in DWELL:
            sign = 1 if y > 0 else -1
            pad = CHARGER["pad_size_m"]
            cabinet = CHARGER["cabinet_size_m"]
            offset = LANE_WIDTH / 2 + CHARGER["edge_gap_m"] + pad[1] / 2
            box(f"{name} equipment pad", pad, (x, y + sign * offset, pad[2] / 2), "green")
            box(f"{name} charger", cabinet, (x, y + sign * offset, pad[2] + cabinet[2] / 2), "green")
            label("CHARGER", x, y + sign * (offset + pad[1] / 2 + 0.425) - 0.12, 0.37)
    label("AGV GUIDEPath".upper(), 0, 2.3, 1.12)
    label("ONE-WAY LOOP", 0, 1.25, 0.48)
    for y, name, color in (
        (-0.3, "TUGGER + TWO CARTS", "amber"),
        (-1.2, "UNIT-LOAD AGV", "blue"),
        (-2.1, "FORKLIFT AGV", "red"),
    ):
        box(name, (0.44, 0.44, 0.035), (-3.65, y + 0.17, 0.025), color)
        label(name, 0, y, 0.44)


def carrier(name, length, width, color):
    body = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(body)
    box(f"{name} chassis", (length, width, 0.27), (0, 0, 0.31), color, body)
    for x in (-length * 0.30, length * 0.30):
        for y in (-width * 0.39, width * 0.39):
            box(f"{name} wheel", (0.25, width * 0.20, 0.28), (x, y, 0.15), "ink", body)
    return body


def build_vehicle(vehicle):
    name = vehicle.name
    length, width, _ = REFERENCE_SIZE[name]
    if name == "Tugger":
        tug = carrier(name, 1.35, width, "amber")
        box("Tugger enclosure", (0.66, 0.67, 0.40), (0.24, 0, 0.63), "amber", tug)
        parts = [tug]
        for index in range(1, len(REFERENCE_OFFSETS[name])):
            cart = carrier(f"Cart {index}", 1.2, 0.9, "amber")
            box("Drawbar", (0.45, 0.10, 0.08), (0.72, 0, 0.22), "ink", cart)
            for y in (-0.20, 0.20):
                box("Bin", (0.83, 0.33, 0.34), (0, y, 0.62), "blue", cart)
            parts.append(cart)
        return parts
    if name == "Unit load":
        deck = carrier(name, length, width, "blue")
        for i in range(7):
            box("Deck roller", (0.10, 0.84, 0.07), (-0.63 + i * 0.20, 0, 0.48), "line", deck)
        box("Unit load", (0.86, 0.72, 0.42), (-0.14, 0, 0.74), "blue", deck)
        return [deck]
    forklift = carrier(name, 1.65, width, "red")
    box("Counterweight", (0.84, 0.90, 0.57), (-0.30, 0, 0.68), "red", forklift)
    for y in (-0.27, 0.27):
        box("Mast", (0.13, 0.13, 1.75), (0.66, y, 1.01), "ink", forklift)
        box("Fork", (1.0, 0.12, 0.07), (1.325, y, 0.14), "amber", forklift)
    return [forklift]


def animate(trace):
    for vehicle in VEHICLES:
        objects = build_vehicle(vehicle)
        parts = zip(objects, BODY_OFFSETS[vehicle.name], strict=True)
        parts = [(obj, offset) for obj, (_, offset) in parts]
        for obj, _ in parts:
            obj.scale = vehicle_scale(vehicle)
        moves = [(t, point) for t, kind, point, name in trace
                 if kind == "enter" and name == vehicle.name]
        index = -1
        previous_angles = [None] * len(parts)
        for tick in range(SECONDS * FPS + 1):
            time = tick / FPS
            while index + 1 < len(moves) and moves[index + 1][0] <= time:
                index += 1
            source = vehicle.start
            distance = 0
            if index >= 0:
                departure, source = moves[index]
                distance = min((time - departure) * vehicle.speed, SEGMENTS[source].length)
            for j, (obj, offset) in enumerate(parts):
                x, y, angle = pose_behind(source, distance - offset)
                previous = previous_angles[j]
                if previous is not None:
                    angle = previous + math.remainder(angle - previous, math.tau)
                previous_angles[j] = angle
                obj.location = (x, y, 0)
                obj.rotation_euler.z = angle
                obj.keyframe_insert(data_path="location", frame=tick + 1)
                obj.keyframe_insert(data_path="rotation_euler", index=2, frame=tick + 1)


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.render.engine = "BLENDER_WORKBENCH"
    shading = scene.display.shading
    shading.color_type = "OBJECT"
    shading.light = "STUDIO"
    shading.show_shadows = True
    shading.show_cavity = True
    shading.cavity_type = "BOTH"
    shading.background_type = "WORLD"
    scene.world = bpy.data.worlds.new("Backdrop")
    scene.world.color = (0.12, 0.15, 0.20)
    scene.render.resolution_x = 1440
    scene.render.resolution_y = 1000
    scene.render.fps = FPS
    scene.frame_end = SECONDS * FPS + 1
    scene.view_settings.view_transform = "Standard"
    build_track()
    animate(simulate(SECONDS))
    framing = max(FLOOR_WIDTH / 40, FLOOR_DEPTH / 26)
    bpy.ops.object.camera_add(location=(0, -12 * framing, 39 * framing))
    camera = bpy.context.object
    camera.name = "Guidepath overview"
    camera.rotation_euler = (-camera.location).to_track_quat("-Z", "Y").to_euler()
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 42 * framing
    scene.camera = camera
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == "VIEW_3D":
                space = area.spaces.active
                space.region_3d.view_perspective = "CAMERA"
                space.shading.color_type = "OBJECT"
                space.overlay.show_overlays = False
    scene.frame_set(1)
    scene.render.filepath = str(HERE / "AGVSystem.png")
    bpy.context.preferences.filepaths.save_version = 0
    scene["agv_dimensions"] = json.dumps(CONFIG, sort_keys=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(HERE / "AGVSystem.blend"))
    export_panda()
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    main()
