"""Live split-screen Panda3D view of the RMFS DEVS example."""

from collections import defaultdict
from itertools import pairwise
from math import prod
from time import perf_counter, sleep


from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import CardMaker, ClockObject, LineSegs, OrthographicLens, TextNode, loadPrcFileData
from socs.devs.TESEngin import TESEnginBase

from examples.AGVSystem.run_panda import add_mesh

from .model import BINS_PER_RACK, CLUSTERS, WORKSTATIONS, OrderAllocator, RMFS
from .traffic_layout import LANE_SEGMENTS, MID_SEGMENT_CONTROL_POINTS, RACK_POSITIONS, STATION_APPROACHES


from .motion import final_heading
from .fleet import dynamic_route
from .playback import bind_speed, speed_label


def box(parent, name, center, size, color):
    x, y, z = center
    w, d, h = (side / 2 for side in size)
    vertices = [(x+i*w, y+j*d, z+k*h) for k in (-1, 1)
                for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    triangles = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
                 (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
                 (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    add_mesh(parent, dict(name=name, vertices=vertices, triangles=triangles, color=color))


EMPTY_ROUTE_COLOR = (1, .91, .24, 1)
ROUTE_COLORS = {
    "TravelToRack": EMPTY_ROUTE_COLOR,
    "TravelToWorkstation": (.33, 1, .45, 1),
    "ReturnRack": (1, .53, .18, 1),
    "ReturnToStandby": EMPTY_ROUTE_COLOR,
}


def draw_route(parent, phase, route, *, color=None):
    path = LineSegs(phase)
    path.setColor(*(ROUTE_COLORS[phase] if color is None else color))
    path.setThickness(3.5)
    path.moveTo(*route[0], .025)
    for point in route[1:]:
        path.drawTo(*point, .025)
    return parent.attachNewNode(path.create())


def draw_composite_routes(parent, routes, colors):
    """Draw shared lane segments once using RGB screen blending."""
    edges = defaultdict(set)
    points = defaultdict(set)
    for robot, route in enumerate(routes):
        for point in route:
            points[point].add(robot)
        for edge in pairwise(route):
            edges[tuple(sorted(edge))].add(robot)

    def blend(owners):
        return tuple(
            1 - prod(1 - colors[robot][channel] for robot in sorted(owners))
            for channel in range(3)
        ) + (1,)

    group = parent.attachNewNode("Robot routes")
    shared = defaultdict(list)
    for edge, owners in edges.items():
        draw_route(group, "TravelToWorkstation", edge, color=blend(owners))
        if len(owners) > 1:
            for point in edge:
                shared[point].append(owners)
    for (x, y), owners in points.items():
        if len(owners) > 1 and owners not in shared[x, y]:
            box(group, "Shared control point", (x, y, .06), (.28, .28, .08), blend(owners))
    return group


class Viewer(ShowBase):
    def update_remaining_routes(self, index):
        routes = self.route_progress.remaining(index)
        if routes != self.visible_routes:
            for path in self.paths:
                path.removeNode()
            self.paths = [draw_composite_routes(self.render, routes, self.route_colors)]
            self.visible_routes = routes

    def __init__(self):
        super().__init__()
        if not hasattr(self, "speed"):
            self.speed = 1.
        bind_speed(self, lambda: self.advance(0.))
        clock = ClockObject.getGlobalClock()
        clock.setMode(ClockObject.MNormal)
        self.frame_started = perf_counter()
        self.taskMgr.add(self.pace_frame, "rmfs-frame-pacing", sort=100)
        self.disableMouse()
        self.setBackgroundColor(.07, .13, .17)
        self.cam.node().getDisplayRegion(0).setDimensions(0, .70, 0, 1)
        self.camLens.setAspectRatio(self.getAspectRatio() * .70)
        self.iso_lens = self.camLens
        self.top_lens = OrthographicLens()
        self.view_zoom = 1.0
        self.view_pan = (.12, 0.0)
        self.set_view("iso")
        for key, dx, dy in (("left", -.06, 0), ("right", .06, 0),
                            ("up", 0, .06), ("down", 0, -.06)):
            self.accept(f"arrow_{key}", self.pan_view, [dx, dy])
            self.accept(f"arrow_{key}-repeat", self.pan_view, [dx, dy])
        for key, mode in (("t", "top"), ("i", "iso")):
            self.accept(key, self.set_view, [mode])
        for key, factor in (("=", 1.2), ("+", 1.2), ("-", 1 / 1.2)):
            self.accept(key, self.zoom_view, [factor])
        self.allocator = OrderAllocator()
        self.assignment = self.allocator.assign()
        self.route_lines = {}
        self.build_scene()
        self.fit_iso_view()
        panel = CardMaker("monitor")
        panel.setFrame(.40, 1, -1, 1)
        background = self.render2d.attachNewNode(panel.generate())
        background.setColor(.05, .10, .13, 1)
        background.setBin("background", 0)
        self.monitor = OnscreenText(
            parent=self.render2d, pos=(.445, .83), scale=(.029, .048),
            align=TextNode.ALeft, fg=(.87, .95, .94, 1), mayChange=True,
        )
        self.accept("space", self.toggle_pause)
        self.accept("r", self.reset)
        self.accept("n", self.next_order)
        self.accept("escape", self.userExit)
        self.reset()
        self.taskMgr.add(self.update, "rmfs-animation")

    def pace_frame(self, task):
        if self.win is None or not self.win.getProperties().getOpen():
            self.taskMgr.stop()
            return task.done
        frame_interval = 1 / (15 if self.paused or self.win.getProperties().getMinimized() else 60)
        sleep(max(0, frame_interval - (perf_counter() - self.frame_started)))
        self.frame_started = perf_counter()
        return task.cont

    def run(self):
        self.frame_started = perf_counter()
        try:
            super().run()
        finally:
            self.destroy()

    def set_view(self, mode):
        self.view_mode = mode
        if mode == "top":
            self.cam.node().setLens(self.top_lens)
            self.camera.setPos(21, 0, 85)
            self.camera.setHpr(0, -90, 0)
        else:
            self.cam.node().setLens(self.iso_lens)
            if hasattr(self, "iso_target"):
                self.fit_iso_view()
                return
        self.zoom_view(1)

    def zoom_view(self, factor):
        self.view_zoom = max(.4, min(4, self.view_zoom * factor))
        if self.view_mode == "top":
            aspect = .70 * self.getAspectRatio()
            width = max(46, 44 * aspect) / self.view_zoom
            self.top_lens.setFilmSize(width, width / aspect)
        else:
            target = getattr(self, "iso_target", (21, 0, 0))
            distance = getattr(self, "iso_fit", 1.25) / self.view_zoom
            x, y, z = target
            self.camera.setPos(x - 36 * distance, y - 49 * distance, z + 61 * distance)
            self.camera.lookAt(*target)
        self.apply_view_pan()

    def pan_view(self, dx, dy):
        x, y = self.view_pan
        self.view_pan = (x + dx, y + dy)
        self.apply_view_pan()

    def apply_view_pan(self):
        lens = self.cam.node().getLens()
        film = lens.getFilmSize()
        x, y = self.view_pan
        if self.view_mode == "iso":
            base_x, base_y = getattr(self, "iso_shift", (0, 0))
            x += base_x
            y += base_y
        lens.setFilmOffset(-x * film.x / 2, -y * film.y / 2)

    def fit_iso_view(self):
        from math import radians, tan
        from panda3d.core import Point2, Point3

        low, high = self.render.getTightBounds()
        self.iso_target = tuple((low[i] + high[i]) / 2 for i in range(3))
        self.iso_fit = 1
        if self.view_mode != "iso":
            return
        self.zoom_view(1)
        target_depth = self.camera.getRelativePoint(self.render, Point3(*self.iso_target)).y
        self.iso_lens.setFilmOffset(0, 0)
        half_x, half_y = (tan(radians(fov / 2)) * 1.135 for fov in self.iso_lens.getFov())
        corners = []
        required = 0.0
        for x in (low.x, high.x):
            for y in (low.y, high.y):
                for z in (low.z, high.z):
                    corners.append(Point3(x, y, z))
                    point = self.camera.getRelativePoint(self.render, Point3(x, y, z))
                    required = max(required, max(abs(point.x) / half_x, abs(point.z) / half_y)
                                   - (point.y - target_depth))
        self.iso_fit = required / (target_depth * self.view_zoom)
        zoom = self.view_zoom
        self.view_zoom = 1
        self.zoom_view(1)
        self.iso_lens.setFilmOffset(0, 0)
        projected = []
        for corner in corners:
            point = Point2()
            self.iso_lens.project(self.camera.getRelativePoint(self.render, corner), point)
            projected.append(point)
        shift_x = min(.025, .998 - max(point.x for point in projected))
        shift_y = min(.14, .998 - max(point.y for point in projected))
        self.iso_shift = (shift_x, shift_y)
        self.view_zoom = zoom
        self.zoom_view(1)

    def windowEvent(self, window):
        # Keep ShowBase's close handling while adjusting our split viewport.
        super().windowEvent(window)
        if window == self.win and window.getYSize() and hasattr(self, "iso_lens"):
            self.iso_lens.setAspectRatio(.70 * window.getXSize() / window.getYSize())
            if self.view_mode == "iso" and hasattr(self, "iso_target"):
                self.fit_iso_view()
            else:
                self.zoom_view(1)

    def build_scene(self):
        yellow = (.87, .66, .20, 1)
        teal = (.27, .48, .53, 1)
        box(self.render, "floor", (21, 1, -.15), (47, 40, .25), (.21, .29, .32, 1))
        self.racks = {}
        for cluster in range(CLUSTERS):
            for row in range(6):
                for col in range(2):
                    x, y = RACK_POSITIONS[cluster, row, col]
                    rack = self.render.attachNewNode(f"C{cluster+1:02}-{row+1}-{col+1}")
                    rack.setPos(x, y, 0)
                    self.racks[cluster, row, col] = rack
                    color = yellow
                    box(rack, "roof", (0, 0, 1.91), (1.5, 1.5, .08), color)
                    for leg_x in (-.70, .70):
                        for leg_y in (-.70, .70):
                            box(rack, "post", (leg_x, leg_y, .95), (.1, .1, 1.9), color)
                    for tier in range(BINS_PER_RACK // 2):
                        z = .49 + .34 * tier
                        box(rack, "shelf", (0, 0, z), (1.44, 1.44, .07), color)
                        for bx in (-.34, .34):
                            bin_node = rack.attachNewNode("bin")
                            box(bin_node, "bin", (bx, 0, z+.12), (.61, 1.20, .17), teal)
        for name, (x, y) in STATION_APPROACHES.items():
            box(self.render, name, (x, y + .9, .45),
                (2.8, .8, .9), teal)
        lanes = LineSegs("Bidirectional lanes")
        lanes.setColor(.62, .71, .72, 1)
        lanes.setThickness(1.5)
        for segment in LANE_SEGMENTS:
            lanes.moveTo(*segment.start, .01)
            lanes.drawTo(*segment.end, .01)
        self.render.attachNewNode(lanes.create())
        for x, y in MID_SEGMENT_CONTROL_POINTS:
            box(self.render, "Mid-segment control point", (x, y, .035),
                (.22, .22, .04), (.94, .49, .27, 1))
        self.robot = self.render.attachNewNode("MiR600")
        box(self.robot, "chassis", (0, 0, .161), (1.35, .91, .322), (.10, .43, .57, 1))
        box(self.robot, "front", (.61, 0, .28), (.13, .8, .08), (.53, .93, 1, 1))
        self.lights()

    def lights(self):
        from panda3d.core import AmbientLight, DirectionalLight

        ambient = AmbientLight("ambient")
        ambient.setColor((.65, .65, .65, 1))
        self.render.setLight(self.render.attachNewNode(ambient))
        sun = DirectionalLight("key")
        sun.setColor((.54, .54, .54, 1))
        light = self.render.attachNewNode(sun)
        light.setHpr(-35, -58, 0)
        self.render.setLight(light)

    def next_order(self):
        if self.model.controller.phase == "Done":
            self.rack.clearColorScale()
            self.assignment = self.allocator.assign()
            self.reset()

    def reset(self):
        self.model = RMFS(self.assignment)
        self.rack = self.racks[self.assignment.rack]
        self.rack.setColorScale(1, .72, .55, 1)
        self.engine = TESEnginBase(self.model)
        self.engine.bVerbose = False
        self.time = 0
        self.paused = False
        self.events = []
        self.advance(0)

    def toggle_pause(self):
        self.paused = not self.paused

    def advance(self, elapsed):
        if elapsed > 0:
            next_event = self.engine.pick_next_execution_time_type()[0]
            if self.speed == float("inf"):
                if next_event < float("inf"):
                    self.time = next_event
            else:
                self.time += min(.1, elapsed) * self.speed
        while self.engine.pick_next_execution_time_type()[0] <= self.time + 1e-9:
            self.engine.execute_one_step()
            self.events.append(f"{self.engine.tNow:5.1f}s  {self.model.robot.phase}")
        if self.model.controller.phase == "Done":
            self.time = self.engine.tNow
        phase = self.model.robot.phase
        motion = self.model.motion
        route = dynamic_route(motion, phase, self.time - self.model.robot.tLast) if phase in motion.routes else ()
        key = (phase, route)
        if key != getattr(self, "_visible_single_route", None):
            for line in self.route_lines.values():
                line.removeNode()
            self.route_lines.clear()
            if len(route) > 1:
                self.route_lines[phase] = draw_route(self.render, phase, route)
            self._visible_single_route = key
        if phase in motion.routes:
            position, heading = motion.pose(phase, self.time - self.model.robot.tLast)
        else:
            position, heading = {
                "LiftRack": (self.model.slot, motion.start_headings["TravelToWorkstation"]),
                "PickBin": (self.model.station, motion.start_headings["ReturnRack"]),
                "LowerRack": (self.model.slot, motion.start_headings["ReturnToStandby"]),
                "Done": (self.model.selected_robot.position, final_heading(motion.routes["ReturnToStandby"])),
            }.get(phase, (self.model.selected_robot.position, -90))
        self.robot.setPos(*position, 0)
        self.robot.setH(heading)
        carrying = phase in {"TravelToWorkstation", "PickBin", "ReturnRack", "LowerRack"}
        # The carried rack translates with the robot while retaining its floor heading.
        self.rack.setPos(*(position if carrying else self.model.slot), .10 if carrying else 0)
        shipped = phase in {"ReturnRack", "LowerRack", "ReturnToStandby", "Done"}
        self.monitor.setText(
            f"RMFS CONTROLLER\n\nSimulation   {self.time:.1f} s\n"
            f"Controller   {self.model.controller.phase}\n{self.model.robot.name}   {phase}\n"
            f"Rack   {'On robot' if carrying else 'At slot'}\n"
            f"Shipped orders   {int(shipped)} / 1\n\n{self.assignment.rack_id} / {self.assignment.workstation}\n"
            "10 clusters / 120 racks / 10 stations\n\nRECENT EVENTS\n"
            + "\n".join(self.events[-6:]) + "\n\nSpace  Pause    R  Replay    N  Next order"
            + f"\nU/D Speed   {speed_label(self.speed)}"
        )

    def update(self, task):
        if not self.paused:
            self.advance(ClockObject.getGlobalClock().getDt())
        return task.cont


def main():
    loadPrcFileData("", "win-size 1320 800\nwindow-title RMFS | Live DEVS")
    Viewer().run()


if __name__ == "__main__":
    main()












