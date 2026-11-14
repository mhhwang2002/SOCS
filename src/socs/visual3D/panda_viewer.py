"""Panda3D renderer that applies published SOCS visual states each frame."""


class Panda3DViewer:
    """Apply a VisualStateBuffer to registered Panda3D scene nodes.

    Importing this module does not require Panda3D; the optional dependency is
    loaded only when a viewer is created.
    """

    def __init__(self, state_buffer, title="SOCS 3D Visualization"):
        try:
            from direct.showbase.ShowBase import ShowBase
            from panda3d.core import (
                AmbientLight,
                DirectionalLight,
                WindowProperties,
            )
        except ImportError as exc:
            raise ImportError(
                "Panda3D is required for the 3D viewer. Install SOCS with "
                "the 'visual3d' extra."
            ) from exc

        self._base = ShowBase()
        self._state_buffer = state_buffer
        self._nodes = {}
        self._overlay_texts = {}
        self._revision = -1
        if self._base.win is not None:
            properties = WindowProperties()
            properties.setTitle(title)
            self._base.win.requestProperties(properties)
        self._base.taskMgr.add(self._update_scene, "socs-visual-state-update")
        self._base.setBackgroundColor(0.32, 0.36, 0.39, 1.0)

        ambient = AmbientLight("socs-ambient")
        ambient.setColor((0.62, 0.62, 0.62, 1.0))
        ambient_node = self._base.render.attachNewNode(ambient)
        self._base.render.setLight(ambient_node)

        key_light = DirectionalLight("socs-key-light")
        key_light.setColor((0.9, 0.88, 0.82, 1.0))
        key_node = self._base.render.attachNewNode(key_light)
        key_node.setHpr(-35, -55, 0)
        self._base.render.setLight(key_node)

    def attach_entity(self, entity_id, node_path):
        """Register an existing Panda3D NodePath for a SOCS entity id."""
        if not entity_id:
            raise ValueError("entity_id must not be empty.")
        if entity_id in self._nodes:
            raise ValueError(f"Entity {entity_id!r} is already registered.")
        self._nodes[entity_id] = node_path
        self._revision = -1

    def load_entity(self, entity_id, model_path, parent=None):
        """Load and register a Panda3D model, returning its NodePath."""
        if parent is None:
            parent = self._base.render
        node_path = self._base.loader.loadModel(model_path)
        if node_path.isEmpty():
            raise FileNotFoundError(f"Panda3D model not found: {model_path}")
        node_path.reparentTo(parent)
        self.attach_entity(entity_id, node_path)
        return node_path

    def set_camera(self, position, target=(0.0, 0.0, 0.0), field_of_view=55.0):
        """Position the camera and point it at a target in world coordinates."""
        from panda3d.core import Mat4

        self._base.disableMouse()
        self._base.camLens.setFov(field_of_view)
        self._base.camera.setPos(*position)
        self._base.camera.lookAt(*target)

        # Trackball stores the inverse camera transform; keep it in sync so
        # the default mouse-control task does not restore an older view.
        camera_transform = Mat4(self._base.camera.getMat())
        camera_transform.invertInPlace()
        self._base.mouseInterfaceNode.setMat(camera_transform)
        self._base.enableMouse()

    def add_polyline(self, points, color=(0.7, 0.7, 0.7, 1.0), thickness=2.0):
        """Draw a fixed world-space line through a sequence of 3D points."""
        from panda3d.core import LineSegs

        points = tuple(points)
        if len(points) < 2:
            raise ValueError("A polyline requires at least two points.")
        lines = LineSegs()
        lines.setColor(*color)
        lines.setThickness(thickness)
        lines.moveTo(*points[0])
        for point in points[1:]:
            lines.drawTo(*point)
        return self._base.render.attachNewNode(lines.create())

    def add_box(self, name, position, size, color, parent=None):
        """Create a static colored box in the scene, returning its NodePath."""
        if parent is None:
            parent = self._base.render
        return self._create_box(name, position, size, color, parent)

    def create_box_entity(self, entity_id, size, parent=None):
        """Create and register a centered, untextured box for a moving entity."""
        if parent is None:
            parent = self._base.render
        node = self._create_box(entity_id, (0.0, 0.0, 0.0), size, (1, 1, 1, 1), parent)
        self.attach_entity(entity_id, node)
        return node

    def _create_box(self, name, position, size, color, parent):
        """Build a unit cube centered at its local origin, without a texture."""
        from panda3d.core import (
            Geom,
            GeomNode,
            GeomTriangles,
            GeomVertexData,
            GeomVertexFormat,
            GeomVertexWriter,
            NodePath,
        )

        vertex_data = GeomVertexData(
            name, GeomVertexFormat.getV3n3c4(), Geom.UHStatic
        )
        vertex_writer = GeomVertexWriter(vertex_data, "vertex")
        normal_writer = GeomVertexWriter(vertex_data, "normal")
        color_writer = GeomVertexWriter(vertex_data, "color")
        triangles = GeomTriangles(Geom.UHStatic)

        faces = (
            (((0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5)), (1, 0, 0)),
            (((-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5)), (-1, 0, 0)),
            (((0.5, 0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5)), (0, 1, 0)),
            (((-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5)), (0, -1, 0)),
            (((-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5)), (0, 0, 1)),
            (((-0.5, 0.5, -0.5), (0.5, 0.5, -0.5), (0.5, -0.5, -0.5), (-0.5, -0.5, -0.5)), (0, 0, -1)),
        )
        for face_index, (vertices, normal) in enumerate(faces):
            first = face_index * 4
            for vertex in vertices:
                vertex_writer.addData3f(*vertex)
                normal_writer.addData3f(*normal)
                color_writer.addData4f(1.0, 1.0, 1.0, 1.0)
            triangles.addVertices(first, first + 1, first + 2)
            triangles.addVertices(first, first + 2, first + 3)
            triangles.closePrimitive()

        geometry = Geom(vertex_data)
        geometry.addPrimitive(triangles)
        geometry_node = GeomNode(name)
        geometry_node.addGeom(geometry)
        node = NodePath(geometry_node)
        node.reparentTo(parent)
        node.setPos(*position)
        node.setScale(*size)
        node.setColorScale(*color)
        return node

    def bind_key(self, key, callback):
        """Bind a keyboard key to a callback on Panda3D's main thread."""
        self._base.accept(key, callback)

    def add_overlay_text(
        self,
        text_id,
        text,
        position=(-1.28, 0.90),
        scale=0.045,
        color=(1.0, 1.0, 1.0, 1.0),
        align="left",
    ):
        """Create screen-space status text updated on the renderer thread."""
        from direct.gui.OnscreenText import OnscreenText
        from panda3d.core import TextNode

        if not text_id or text_id in self._overlay_texts:
            raise ValueError(f"Overlay text id must be unique: {text_id!r}.")
        align_value = {
            "left": TextNode.ALeft,
            "center": TextNode.ACenter,
            "right": TextNode.ARight,
        }.get(align.lower())
        if align_value is None:
            raise ValueError("align must be 'left', 'center', or 'right'.")
        node = OnscreenText(
            text=text,
            pos=position,
            scale=scale,
            fg=color,
            align=align_value,
            mayChange=True,
            shadow=(0.0, 0.0, 0.0, 0.8),
            shadowOffset=(0.04, 0.04),
            bg=(0.05, 0.07, 0.09, 0.68),
            wordwrap=70,
        )
        self._overlay_texts[text_id] = node
        return node

    def add_world_text(
        self,
        text,
        position,
        scale=0.4,
        color=(1.0, 1.0, 1.0, 1.0),
        align="center",
        parent=None,
    ):
        """Create camera-facing text at a fixed position in the 3D scene."""
        from panda3d.core import NodePath, TextNode

        if parent is None:
            parent = self._base.render
        text_node = TextNode(f"socs-world-text-{len(self._nodes)}")
        text_node.setText(str(text))
        text_node.setTextColor(*color)
        text_node.setCardColor(0.05, 0.07, 0.09, 0.95)
        text_node.setCardAsMargin(0.08, 0.08, 0.05, 0.05)
        text_node.setCardDecal(True)
        align_value = {
            "left": TextNode.ALeft,
            "center": TextNode.ACenter,
            "right": TextNode.ARight,
        }.get(align.lower())
        if align_value is None:
            raise ValueError("align must be 'left', 'center', or 'right'.")
        text_node.setAlign(align_value)
        node = NodePath(text_node)
        node.reparentTo(parent)
        node.setPos(*position)
        node.setBillboardPointEye()
        node.setScale(scale)
        return node

    def set_overlay_text(self, text_id, text):
        """Update an existing screen-space text element."""
        try:
            self._overlay_texts[text_id].setText(text)
        except KeyError as exc:
            raise KeyError(f"Unknown overlay text id: {text_id!r}.") from exc

    def bind_overlay_provider(self, text_id, provider):
        """Refresh an overlay from a thread-safe text provider each frame."""
        if text_id not in self._overlay_texts:
            raise KeyError(f"Unknown overlay text id: {text_id!r}.")
        if not callable(provider):
            raise TypeError("provider must be callable.")

        def update_overlay(task):
            self.set_overlay_text(text_id, provider())
            return task.cont

        self._base.taskMgr.add(update_overlay, f"overlay-update-{text_id}")

    def run(self):
        """Run Panda3D's window and render loop."""
        self._base.run()

    def close(self):
        """Stop Panda3D's main loop and destroy its window."""
        self._base.userExit()

    def _update_scene(self, task):
        revision, _simulation_time, entities = self._state_buffer.read()
        if revision == self._revision:
            return task.cont

        for entity_id, entity in entities.items():
            node = self._nodes.get(entity_id)
            if node is None:
                continue

            node.setPos(*entity.position)
            node.setH(entity.heading)
            if entity.color is not None:
                node.setColorScale(*entity.color)
            if entity.visible:
                node.show()
            else:
                node.hide()

        for entity_id, node in self._nodes.items():
            if entity_id not in entities:
                node.hide()

        self._revision = revision
        return task.cont
