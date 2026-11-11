"""Export the editable Blender scene's geometry for the Panda3D viewer."""
import json
import sys
from pathlib import Path

import bpy
from mathutils import Matrix

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from examples.AGVSystem.layout import BODY_OFFSETS, CONFIG, VEHICLES, vehicle_scale


def main():
    scene = bpy.context.scene
    scene.frame_set(1)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bodies = {name for parts in BODY_OFFSETS.values() for name, _ in parts}
    for name in bodies:
        if scene.objects.get(name) is None:
            raise ValueError(f"Missing Blender body {name}")

    dimensions = json.dumps(CONFIG, sort_keys=True)
    if scene.get("agv_dimensions") != dimensions:
        raise ValueError("Dimensions changed; rebuild AGVSystem.blend before exporting")
    scales = {name: vehicle_scale(v) for v in VEHICLES for name, _ in BODY_OFFSETS[v.name]}
    geometry = []
    for obj in scene.objects:
        if obj.type not in {"MESH", "FONT"}:
            continue
        root = obj.parent
        while root is not None and root.name not in bodies:
            root = root.parent
        evaluated = obj.evaluated_get(depsgraph)
        matrix = evaluated.matrix_world
        if root is not None:
            matrix = Matrix.Diagonal((*scales[root.name], 1)) @ root.matrix_world.inverted() @ matrix
        mesh = evaluated.to_mesh()
        try:
            mesh.calc_loop_triangles()
            geometry.append({
                "name": obj.name,
                "body": root.name if root else None,
                "color": list(obj.color),
                "vertices": [list(matrix @ vertex.co) for vertex in mesh.vertices],
                "triangles": [list(triangle.vertices) for triangle in mesh.loop_triangles],
            })
        finally:
            evaluated.to_mesh_clear()

    (HERE / "panda_scene.json").write_text(
        json.dumps({"dimensions": CONFIG, "geometry": geometry}, separators=(",", ":")), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
