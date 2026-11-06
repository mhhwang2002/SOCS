"""Panda3D renderer that applies published SOCS visual states each frame."""


class Panda3DViewer:
    """Apply a VisualStateBuffer to registered Panda3D scene nodes.

    Importing this module does not require Panda3D; the optional dependency is
    loaded only when a viewer is created.
    """

    def __init__(self, state_buffer, title="SOCS 3D Visualization"):
        try:
            from direct.showbase.ShowBase import ShowBase
            from panda3d.core import WindowProperties
        except ImportError as exc:
            raise ImportError(
                "Panda3D is required for the 3D viewer. Install SOCS with "
                "the 'visual3d' extra."
            ) from exc

        self._base = ShowBase()
        self._state_buffer = state_buffer
        self._nodes = {}
        self._revision = -1
        if self._base.win is not None:
            properties = WindowProperties()
            properties.setTitle(title)
            self._base.win.requestProperties(properties)
        self._base.taskMgr.add(self._update_scene, "socs-visual-state-update")

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

    def set_camera(self, position, target=(0.0, 0.0, 0.0)):
        """Position the camera and point it at a target in world coordinates."""
        self._base.camera.setPos(*position)
        self._base.camera.lookAt(*target)

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

    def bind_key(self, key, callback):
        """Bind a keyboard key to a callback on Panda3D's main thread."""
        self._base.accept(key, callback)

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
