"""Build an editable AS/RS warehouse and a short crane/fork motion cycle."""

import math
import os

import bpy
from mathutils import Vector


OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
BLEND_PATH = os.path.join(OUTPUT_DIR, "ASRS_warehouse.blend")
PREVIEW_PATH = os.path.join(OUTPUT_DIR, "ASRS_warehouse_preview.png")

CELL_WIDTH = 1.0
LEVEL_PITCH = 0.75
BEAM_SIZE = 0.05
RACK_COLUMNS = 15
RACK_LEVELS = 6
RACK_DEPTH = 1.0
MAX_FORK_REACH = 1.25
FORK_LIFT_CLEARANCE = 0.12
TARGET_BAY = 8
TARGET_LEVEL = 4


def make_material(name, color, metallic=0.0, roughness=0.65):
    material = bpy.data.materials.new(name)
    material.diffuse_color = (*color, 1.0)
    principled = material.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = (*color, 1.0)
    principled.inputs["Metallic"].default_value = metallic
    principled.inputs["Roughness"].default_value = roughness
    return material


def make_box(name, dimensions, location, material, parent=None, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.0, 0.0, 0.0))
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if parent is None:
        obj.location = location
    else:
        obj.parent = parent
        obj.location = location
    obj.data.materials.append(material)
    if bevel:
        modifier = obj.modifiers.new("Edge softening", "BEVEL")
        modifier.width = bevel
        modifier.segments = 2
        obj.modifiers.new("Weighted corner normals", "WEIGHTED_NORMAL")
    return obj


def make_empty(name, location=(0.0, 0.0, 0.0), parent=None, display="PLAIN_AXES"):
    obj = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(obj)
    obj.empty_display_type = display
    obj.empty_display_size = 0.25
    obj.parent = parent
    obj.location = location
    return obj


def animate_location(obj, axis, keys):
    for frame, value in keys:
        obj.location[axis] = value
        obj.keyframe_insert(data_path="location", index=axis, frame=frame)
    if obj.animation_data and obj.animation_data.action:
        action = obj.animation_data.action
        slot = obj.animation_data.action_slot
        for layer in action.layers:
            for strip in layer.strips:
                channelbag = strip.channelbag(slot)
                if channelbag:
                    for curve in channelbag.fcurves:
                        for key in curve.keyframe_points:
                            key.interpolation = "LINEAR"


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for data in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.cameras, bpy.data.lights):
        for item in list(data):
            if item.users == 0:
                data.remove(item)


def build_scene():
    clear_scene()
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "METERS"
    scene.render.fps = 24
    scene.frame_start = 1
    scene.frame_end = 180

    floor_mat = make_material("Concrete floor", (0.34, 0.39, 0.43))
    rack_mat = make_material("Rack uprights", (0.08, 0.24, 0.34), metallic=0.35)
    beam_mat = make_material("Rack beams - 50 mm", (0.06, 0.43, 0.57), metallic=0.25)
    deck_mat = make_material("Shelf decks", (0.24, 0.31, 0.34), metallic=0.2)
    rail_mat = make_material("Crane rails", (0.08, 0.10, 0.12), metallic=0.6)
    crane_mat = make_material("Crane frame", (0.92, 0.36, 0.08), metallic=0.2)
    crane_dark = make_material("Crane dark parts", (0.14, 0.18, 0.20), metallic=0.45)
    fork_mat = make_material("Forks", (0.83, 0.68, 0.22), metallic=0.4)
    load_mat = make_material("Stored loads", (0.68, 0.76, 0.78))

    rack_length = RACK_COLUMNS * CELL_WIDTH
    rack_height = RACK_LEVELS * LEVEL_PITCH
    warehouse_length = rack_length + 3.0
    aisle_y = -1.55

    make_box(
        "Warehouse floor 18 x 6 m",
        (warehouse_length, 6.0, 0.12),
        (rack_length / 2.0, -0.15, -0.06),
        floor_mat,
    )

    rack_root = make_empty("Rack - 15 bays x 6 levels")
    rack_root["bay_width_m"] = CELL_WIDTH
    rack_root["level_pitch_m"] = LEVEL_PITCH
    rack_root["beam_thickness_m"] = BEAM_SIZE
    rack_root["depth_m_assumed"] = RACK_DEPTH
    rack_root["storage_cells"] = RACK_COLUMNS * RACK_LEVELS

    post_size = 0.07
    for column in range(RACK_COLUMNS + 1):
        x = column * CELL_WIDTH
        for y in (0.0, RACK_DEPTH):
            make_box(
                f"Rack upright X{column:02d} Y{int(y)}",
                (post_size, post_size, rack_height),
                (x, y, rack_height / 2.0),
                rack_mat,
                parent=rack_root,
                bevel=0.008,
            )

    for level in range(1, RACK_LEVELS + 1):
        z = level * LEVEL_PITCH - BEAM_SIZE / 2.0
        for y in (0.0, RACK_DEPTH):
            make_box(
                f"Rack beam level {level} face {int(y)}",
                (rack_length, BEAM_SIZE, BEAM_SIZE),
                (rack_length / 2.0, y, z),
                beam_mat,
                parent=rack_root,
                bevel=0.006,
            )
        for column in range(RACK_COLUMNS):
            make_box(
                f"Shelf support L{level} B{column + 1:02d}",
                (BEAM_SIZE, RACK_DEPTH, BEAM_SIZE),
                ((column + 0.5) * CELL_WIDTH, RACK_DEPTH / 2.0, z),
                beam_mat,
                parent=rack_root,
                bevel=0.005,
            )

    for level in range(RACK_LEVELS):
        shelf_z = level * LEVEL_PITCH + 0.035
        for column in range(RACK_COLUMNS):
            make_box(
                f"Shelf deck L{level + 1} B{column + 1:02d}",
                (CELL_WIDTH - 0.09, RACK_DEPTH - 0.12, 0.025),
                ((column + 0.5) * CELL_WIDTH, RACK_DEPTH / 2.0, shelf_z),
                deck_mat,
                parent=rack_root,
            )
            if (column + level) % 3 == 0 and not (
                column == TARGET_BAY - 1 and level == TARGET_LEVEL - 1
            ):
                make_box(
                    f"Stored load L{level + 1} B{column + 1:02d}",
                    (0.78, 0.76, 0.42),
                    ((column + 0.5) * CELL_WIDTH, RACK_DEPTH / 2.0, shelf_z + 0.235),
                    load_mat,
                    parent=rack_root,
                    bevel=0.025,
                )

    rail_length = rack_length + 0.7
    rail_z = 0.08
    for rail_index, y in enumerate((aisle_y - 0.32, aisle_y + 0.32), start=1):
        make_box(
            f"Crane floor rail {rail_index}",
            (rail_length, 0.12, 0.10),
            (rack_length / 2.0, y, rail_z),
            rail_mat,
            bevel=0.015,
        )

    crane_root = make_empty("Crane X - aisle travel", (0.5, aisle_y, 0.15))
    crane_root["axis"] = "X travel along rack"
    make_box("Crane lower chassis", (0.78, 0.76, 0.24), (0.0, 0.0, 0.16), crane_mat, crane_root, 0.025)
    for wheel_x in (-0.27, 0.27):
        for wheel_y in (-0.31, 0.31):
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=24, radius=0.12, depth=0.09, location=(0.0, 0.0, 0.0)
            )
            wheel = bpy.context.object
            wheel.name = f"Crane wheel {wheel_x:+.2f} {wheel_y:+.2f}"
            wheel.rotation_euler[0] = math.radians(90.0)
            wheel.parent = crane_root
            wheel.location = (wheel_x, wheel_y, 0.04)
            wheel.data.materials.append(rail_mat)

    crane_y = make_empty("Crane Y - aisle positioning", (0.0, 0.0, 0.0), crane_root)
    crane_y["axis"] = "Y cross-aisle carriage"
    make_box("Crane mast", (0.16, 0.18, 5.15), (0.0, 0.0, 2.72), crane_mat, crane_y, 0.018)
    make_box("Mast guide left", (0.045, 0.045, 4.85), (-0.16, 0.0, 2.7), crane_dark, crane_y)
    make_box("Mast guide right", (0.045, 0.045, 4.85), (0.16, 0.0, 2.7), crane_dark, crane_y)

    hoist = make_empty(
        "Crane Z - hoist carriage",
        (0.0, 0.0, LEVEL_PITCH * 0.5),
        crane_y,
    )
    hoist["axis"] = "Z lift along mast"
    make_box("Hoist carriage", (0.62, 0.44, 0.23), (0.0, 0.0, 0.0), crane_dark, hoist, 0.02)
    make_box("Fork deck", (0.58, 0.66, 0.08), (0.0, 0.15, -0.13), crane_mat, hoist, 0.012)

    fork_slide = make_empty("Fork ReachOut / Withdraw - Y", (0.0, 0.15, -0.12), hoist)
    fork_slide["axis"] = "Y telescopic reach"
    lift = make_empty("Fork LiftUp / Down - Z", (0.0, 0.0, 0.0), fork_slide)
    for tine_x in (-0.18, 0.18):
        make_box(
            f"Fork tine {tine_x:+.2f}",
            (0.07, 1.20, 0.045),
            (tine_x, 0.50, 0.0),
            fork_mat,
            lift,
            0.012,
        )

    target_x = (TARGET_BAY - 0.5) * CELL_WIDTH
    target_z = (TARGET_LEVEL - 0.5) * LEVEL_PITCH
    make_box(
        "Fork target load - cell L4 B8",
        (0.78, 0.76, 0.42),
        (target_x, RACK_DEPTH / 2.0, (TARGET_LEVEL - 1) * LEVEL_PITCH + 0.27),
        load_mat,
        parent=rack_root,
        bevel=0.025,
    )

    # The crane travels to bay 8 / level 4, then demonstrates the four fork actions.
    animate_location(crane_root, 0, [(1, 0.5), (76, target_x)])
    animate_location(crane_y, 1, [(1, 0.0), (76, 0.18)])
    animate_location(hoist, 2, [(1, LEVEL_PITCH * 0.5), (76, target_z)])
    animate_location(
        fork_slide,
        1,
        [(1, 0.0), (76, 0.0), (88, MAX_FORK_REACH), (104, MAX_FORK_REACH), (116, 0.0)],
    )
    animate_location(
        lift,
        2,
        [(1, 0.0), (88, 0.0), (96, FORK_LIFT_CLEARANCE), (104, 0.0), (116, 0.0)],
    )
    animate_location(crane_root, 0, [(1, 0.5), (76, target_x), (150, target_x), (180, 0.5)])
    animate_location(hoist, 2, [(1, LEVEL_PITCH * 0.5), (76, target_z), (150, target_z), (180, LEVEL_PITCH * 0.5)])

    pallet = make_box(
        "Transfer pallet on forks",
        (0.72, 0.72, 0.18),
        (0.0, 0.58, 0.13),
        fork_mat,
        parent=lift,
        bevel=0.018,
    )
    pallet["purpose"] = "Demonstrates fork reach and vertical clearance"

    make_box(
        "Warehouse back wall",
        (warehouse_length, 0.10, 5.5),
        (rack_length / 2.0, RACK_DEPTH + 0.45, 2.75),
        floor_mat,
    )
    wall = bpy.data.objects.get("Warehouse back wall")
    wall.display_type = "WIRE"
    wall.hide_render = True

    scene.frame_set(100)
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 1000
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = PREVIEW_PATH
    scene.world.color = (0.16, 0.18, 0.21)

    bpy.ops.object.camera_add(location=(20.0, -24.0, 16.5))
    camera = bpy.context.object
    camera.name = "ASRS overview camera"
    target = Vector((7.3, 0.0, 2.2))
    direction = target - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 22.5
    scene.camera = camera

    bpy.ops.object.light_add(type="AREA", location=(7.0, -10.0, 16.0))
    key_light = bpy.context.object
    key_light.name = "Warehouse key light"
    key_light.data.energy = 5000
    key_light.data.shape = "DISK"
    key_light.data.size = 12
    key_light.rotation_euler = (math.radians(18), 0.0, math.radians(25))

    scene.render.image_settings.color_mode = "RGBA"
    scene.view_settings.view_transform = "AgX"
    scene.render.film_transparent = False
    scene.render.filepath = PREVIEW_PATH
    scene.frame_set(100)

    # Store the design assumptions in the project for later editing.
    text = bpy.data.texts.new("ASRS Design Assumptions")
    text.write(
        "AS/RS concept model\n"
        "Source sketch: 1.0 m rack cell width, 0.75 m level pitch, 50 mm beam thickness.\n"
        "Interpreted grid: 15 bays x 6 levels = 90 storage cells.\n"
        "Derived rack envelope: 15.0 m long x 4.5 m high.\n"
        "Assumed because unspecified: 1.0 m rack depth, 2.0 m aisle, 18 m x 6 m floor.\n"
        "Crane axes: X aisle travel, Y aisle positioning, Z hoist.\n"
        "Fork cycle: ReachOut (Y) -> LiftUp (Z) -> Down (Z) -> Withdraw (Y).\n"
        "Assumed fork reach: 1.25 m; vertical clearance: 0.12 m.\n"
        "This is a conceptual kinematic model, not a structural or safety design.\n"
    )

    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == "VIEW_3D":
                area.spaces.active.region_3d.view_perspective = "CAMERA"

    scene.frame_set(1)
    bpy.ops.wm.save_as_mainfile(filepath=BLEND_PATH)
    scene.frame_set(100)
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    build_scene()
