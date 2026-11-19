"""R1 straight run: Space advances one DEVS step; T/I select the camera."""

from argparse import ArgumentParser
from math import atan2, cos, degrees, isfinite, pi, sin

from socs.devs.TESEngin import ExecutionType, TESEnginBase

from ..GridPath import CONTROL_POINTS, POINTS_PER_AXIS, SEGMENTS, SPACING_M
from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

PATH = tuple((ix, POINTS_PER_AXIS // 2) for ix in range(POINTS_PER_AXIS))
EXTENT = (POINTS_PER_AXIS - 1) * SPACING_M
COLLISION_INTERVAL = .01


def path_sweep_rectangles(points, rack_size):
    """Exact fixed-footprint sweep, tiled without overlapping transparency.

    Points and rack dimensions are in metres; segments must be axis aligned.
    Returned rectangles cover the union, including the endpoint footprints.
    """
    points = tuple(tuple(point) for point in points)
    if not points or any(len(p) != 2 or not all(isfinite(x) for x in p) for p in points):
        raise ValueError("Sweep requires finite 2D points.")
    if len(rack_size) != 2 or any(not isfinite(x) or x <= 0 for x in rack_size):
        raise ValueError("Sweep requires positive finite footprint dimensions.")
    hx, hy = (x/2 for x in rack_size)
    boxes = []
    for a,b in zip(points, points[1:] or points):
        if a[0] != b[0] and a[1] != b[1]:
            raise ValueError("Sweep segments must be axis aligned.")
        boxes.append((min(a[0],b[0])-hx, min(a[1],b[1])-hy,
                      max(a[0],b[0])+hx, max(a[1],b[1])+hy))
    xs = sorted({x for box in boxes for x in (box[0],box[2])})
    result = []
    for left,right in zip(xs,xs[1:]):
        intervals = sorted((b[1],b[3]) for b in boxes if b[0] < right and b[2] > left)
        merged = []
        for lower,upper in intervals:
            if merged and lower <= merged[-1][1]:
                merged[-1][1] = max(merged[-1][1],upper)
            else:
                merged.append([lower,upper])
        result.extend((left,lower,right,upper) for lower,upper in merged)
    return tuple(result)


def create_model(path=PATH, *, lift_up=True, rack_size=(1.5, 1.5),
                 name="01_01_RobotPath_Straight", collision_check=False,
                 stop_on_collision=True, initial_headings=None):
    model = RMFS(name, collision_check=collision_check)
    controller = TestController(path)
    robot = MRobot("R1", path[0], lift_up=lift_up, rack_size=rack_size,
                   stop_on_collision=stop_on_collision,
                   initial_heading_deg=(initial_headings or {}).get("R1"))
    sensor = PSensor("S1", stop_on_collision=stop_on_collision)
    model.add_components(controller, robot, sensor)
    model.add_couples(
        ((controller, "!R1"), (robot, "?command")),
        ((robot, "!motion_plan"), (sensor, "?motion_plan")),
        ((sensor, "!sense"), (controller, "?R1")),
        ((sensor, "!Collision"), (controller, "?collision_R1")),
        ((robot, "!arrived"), (controller, "?arrived_R1")),
        ((robot, "!rotation_started"), (controller, "?rotation_started_R1")),
        ((robot, "!rotation_finished"), (controller, "?rotation_finished_R1")),
    )
    return model, robot, sensor


def create_scene(path, name, lift_up, model_factory=None, *,
                 collision_check=False, stop_on_collision=True, initial_headings=None):
    if model_factory is not None:
        return model_factory(lift_up=lift_up, collision_check=collision_check,
                             stop_on_collision=stop_on_collision,
                             initial_headings=initial_headings)
    model, robot, sensor = create_model(
        path, name=name, lift_up=lift_up, collision_check=collision_check,
        stop_on_collision=stop_on_collision, initial_headings=initial_headings,
    )
    return model, (robot,), (sensor,)


def run_headless(path=PATH, *, name="01_01_RobotPath_Straight", lift_up=True,
                 model_factory=None, robot_paths=None, collision_check=False,
                 stop_on_collision=True, initial_headings=None):
    model, robots, sensors = create_scene(
        path, name, lift_up, model_factory, collision_check=collision_check,
        stop_on_collision=stop_on_collision, initial_headings=initial_headings,
    )
    engine = TESEnginBase(
        model, TimeIntervalAnimation=COLLISION_INTERVAL if collision_check else float("inf"),
    )
    engine.bVerbose = False
    while isfinite(model.tNext):
        engine.execute_one_step()
    routes = robot_paths or {"R1": path}
    for robot, sensor in zip(robots, sensors):
        assert robot.phase in ("Wait","Collided")
        if robot.phase == "Wait":
            assert robot._variable.current_position == CONTROL_POINTS[routes[robot.name][-1]]
            assert sensor.phase == "Idle"
        else:
            assert sensor.phase == "Collided"
        assert not sensor._variable.pending_events
    states = "; ".join(
        f"{robot.name}={robot.phase}; {sensor.name}={sensor.phase}"
        for robot, sensor in zip(robots, sensors)
    )
    print(f"Finished at t={engine.tNow:.6f}; {states}")


def main(path=PATH, *, name="01_01_RobotPath_Straight", no_rack=False,
         model_factory=None, robot_paths=None, collision_check=False,
         stop_on_collision=True, initial_headings=None):
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--non-interactive", action="store_true",
                        help="advance automatically to the final point")
    parser.add_argument("--headless", action="store_true",
                        help="run only DEVS without opening a window")
    parser.add_argument("--white-background", action="store_true",
                        help="use a white background with dark text and grid")
    parser.add_argument("--no-rack", action="store_true", default=no_rack,
                        help="run an empty robot using empty motion limits")
    parser.add_argument("--heading", action="append", nargs=2,
                        metavar=("ROBOT", "DEGREES"), default=[],
                        help="initial heading: east 0, north 90; omit to align with the first segment")
    check_options = parser.add_mutually_exclusive_group()
    check_options.add_argument("--collision-check", action="store_true",
                        help="check rack-sized AABBs at animation samples and DEVS advances")
    check_options.add_argument("--no-collision-check", dest="collision_check", action="store_false")
    stop_options = parser.add_mutually_exclusive_group()
    stop_options.add_argument("--no-stop-on-collision", action="store_true",
                        help="turn red during collisions and preserve movement and sensing schedules")
    stop_options.add_argument("--stop-on-collision", dest="no_stop_on_collision", action="store_false")
    parser.set_defaults(collision_check=collision_check,
                        no_stop_on_collision=not stop_on_collision)
    args = parser.parse_args()
    headings = dict(initial_headings or {})
    for robot_name, angle in args.heading:
        try:
            angle = float(angle)
            if not isfinite(angle):
                raise ValueError
        except ValueError:
            parser.error("--heading requires a finite angle in degrees")
        headings[robot_name] = angle
    routes = robot_paths or {"R1": path}
    if set(headings)-set(routes):
        parser.error("Unknown robot in initial headings: " + ", ".join(sorted(set(headings)-set(routes))))
    if args.headless:
        run_headless(path, name=name, lift_up=not args.no_rack,
                     model_factory=model_factory, robot_paths=robot_paths,
                     collision_check=args.collision_check,
                     stop_on_collision=not args.no_stop_on_collision,
                     initial_headings=headings)
        return
    from direct.showbase.ShowBase import ShowBase
    from direct.gui.OnscreenText import OnscreenText
    from panda3d.core import (
        ClockObject, LineSegs, OrthographicLens, TextNode, loadPrcFileData,
        Geom, GeomNode, GeomTriangles, GeomVertexData, GeomVertexFormat,
        GeomVertexWriter,
        TransparencyAttrib,
    )

    def draw_sweep(parent, points, rack_size, color):
        data = GeomVertexData("path sweep", GeomVertexFormat.getV3(), Geom.UHStatic)
        writer = GeomVertexWriter(data,"vertex")
        triangles = GeomTriangles(Geom.UHStatic)
        for index,(left,bottom,right,top) in enumerate(path_sweep_rectangles(points,rack_size)):
            for x,y in ((left,bottom),(right,bottom),(right,top),(left,top)):
                writer.addData3(x,y,.022)
            first = index*4
            triangles.addVertices(first,first+1,first+2)
            triangles.addVertices(first,first+2,first+3)
        geom = Geom(data)
        geom.addPrimitive(triangles)
        node = GeomNode("path sweep")
        node.addGeom(geom)
        result = parent.attachNewNode(node)
        result.setColor(*color[:3],.23)
        result.setTransparency(TransparencyAttrib.MAlpha)
        result.setDepthWrite(False)
        result.setTwoSided(True)
        return result

    def box(parent, center, size, color):
        data = GeomVertexData("box", GeomVertexFormat.getV3(), Geom.UHStatic)
        writer = GeomVertexWriter(data, "vertex")
        for z in (-1, 1):
            for x, y in ((-1,-1),(1,-1),(1,1),(-1,1)):
                writer.addData3(center[0]+x*size[0]/2,
                                center[1]+y*size[1]/2,
                                center[2]+z*size[2]/2)
        triangles = GeomTriangles(Geom.UHStatic)
        for a,b,c in ((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
                      (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)):
            triangles.addVertices(a,b,c)
        geom = Geom(data)
        geom.addPrimitive(triangles)
        node = GeomNode("box")
        node.addGeom(geom)
        result = parent.attachNewNode(node)
        result.setColor(*color)
        return result

    def rounded_chassis(parent):
        # +X is the front. Keep the 1.35 x .91 m bounding rectangle.
        radius = .23
        corner_x = .675-radius
        corner_y = .455-radius
        outline = [(-.675,-.455), (corner_x,-.455)]
        for step in range(1,9):
            angle = -pi/2 + step*pi/16
            outline.append((corner_x+radius*cos(angle),
                            -corner_y+radius*sin(angle)))
        outline.append((.675,corner_y))
        for step in range(1,9):
            angle = step*pi/16
            outline.append((corner_x+radius*cos(angle),
                            corner_y+radius*sin(angle)))
        outline.append((-.675,.455))
        data = GeomVertexData("rounded chassis", GeomVertexFormat.getV3(), Geom.UHStatic)
        writer = GeomVertexWriter(data,"vertex")
        for z in (0,.322):
            for x,y in outline:
                writer.addData3(x,y,z)
        count = len(outline)
        triangles = GeomTriangles(Geom.UHStatic)
        for index in range(1,count-1):
            triangles.addVertices(0,index+1,index)
            triangles.addVertices(count,count+index,count+index+1)
        for index in range(count):
            following = (index+1)%count
            triangles.addVertices(index,following,count+following)
            triangles.addVertices(index,count+following,count+index)
        geom = Geom(data)
        geom.addPrimitive(triangles)
        node = GeomNode("rounded chassis")
        node.addGeom(geom)
        result = parent.attachNewNode(node)
        result.setColor(.18,.72,.30,1)
        return result

    loadPrcFileData("", f"window-title RMFS | {name}")
    loadPrcFileData("", "win-size 1200 850")
    loadPrcFileData("", "sync-video true")

    class Viewer(ShowBase):
        def __init__(self):
            super().__init__()
            self.disableMouse()
            self.setBackgroundColor(*((1,1,1) if args.white_background else (.06,.10,.14)))
            ClockObject.getGlobalClock().setMode(ClockObject.MLimited)
            ClockObject.getGlobalClock().setFrameRate(60)
            self.model, self.robots, self.sensors = create_scene(
                path, name, not args.no_rack, model_factory,
                collision_check=args.collision_check,
                stop_on_collision=not args.no_stop_on_collision,
                initial_headings=headings,
            )
            self.robot, self.sensor = self.robots[0], self.sensors[0]
            self.routes = robot_paths or {self.robot.name: path}
            self.sample_interval = COLLISION_INTERVAL if args.collision_check else float("inf")
            self.engine = TESEnginBase(
                self.model, TimeIntervalAnimation=self.sample_interval,
            )
            self.engine.bVerbose = False
            self.view = "iso"
            self.clock_origin = None
            self.base_time = 0.0
            self.speed = 1.0
            self.finished = False
            self.interactive = not args.non_interactive
            self.rack = self.render.attachNewNode("Rack1")
            self.body = self.render.attachNewNode("R1")
            rounded_chassis(self.body)
            for y in (-.25,.25):
                eye = self.loader.loadModel("models/misc/sphere")
                eye.reparentTo(self.body)
                eye.setPos(.48,y,.38)
                eye.setScale(.24,.21,.24)
                eye.setColor(.9,1,1,1)
                pupil = self.loader.loadModel("models/misc/sphere")
                pupil.reparentTo(self.body)
                pupil.setPos(.67,y,.42)
                pupil.setScale(.075,.115,.13)
                pupil.setColor(.025,.07,.12,1)
            rack_x, rack_y = self.robot.rack_size
            box(self.rack,(0,0,1.91),(rack_x,rack_y,.08),(1,.75,.1,1))
            for x in (-rack_x/2+.05,rack_x/2-.05):
                for y in (-rack_y/2+.05,rack_y/2-.05):
                    box(self.rack,(x,y,1.0),(.1,.1,1.9),(1,.75,.1,1))
            for z in (.5,.84,1.18,1.52):
                box(self.rack,(0,0,z),(rack_x-.06,rack_y-.06,.06),(.95,.7,.1,1))
            if args.no_rack:
                self.rack.hide()
            self.bodies = {self.robot.name: self.body}
            self.racks = {self.robot.name: self.rack}
            for robot in self.robots[1:]:
                self.bodies[robot.name] = self.body.copyTo(self.render)
                self.racks[robot.name] = self.rack.copyTo(self.render)
            route_colors = ((1,0,0,1), (0,1,0,1), (0,0,1,1))
            if len(self.robots) > 1:
                for index, robot in enumerate(self.robots):
                    label = TextNode(robot.name)
                    label.setText(robot.name)
                    label.setTextColor(*route_colors[index % len(route_colors)])
                    node = self.bodies[robot.name].attachNewNode(label)
                    node.setPos(0,0,.9)
                    node.setScale(.45)
                    node.setBillboardPointEye()
            self.points = {}
            grid = LineSegs()
            grid.setColor(*((.55,.6,.63,1) if args.white_background else (.35,.43,.48,1)))
            for start,end in SEGMENTS:
                grid.moveTo(*CONTROL_POINTS[start],.01)
                grid.drawTo(*CONTROL_POINTS[end],.01)
            self.render.attachNewNode(grid.create())
            self.sweeps = {}
            robots_by_name = {robot.name: robot for robot in self.robots}
            for index,(robot_name,route_path) in enumerate(self.routes.items()):
                self.sweeps[robot_name] = draw_sweep(
                    self.render, tuple(CONTROL_POINTS[p] for p in route_path),
                    robots_by_name[robot_name].rack_size,
                    route_colors[index % len(route_colors)],
                )
            for index, route_path in enumerate(self.routes.values()):
                route = LineSegs()
                route.setThickness(4)
                route.setColor(*route_colors[index % len(route_colors)])
                route.moveTo(*CONTROL_POINTS[route_path[0]],.04)
                for point in route_path[1:]:
                    route.drawTo(*CONTROL_POINTS[point],.04)
                self.render.attachNewNode(route.create())
            for point in CONTROL_POINTS:
                x,y = CONTROL_POINTS[point]
                self.points[point] = box(self.render,(x,y,.06),(.13,.13,.1),
                                         (.65,.72,.77,1))
            for point in dict.fromkeys(
                point for route_path in self.routes.values() for point in route_path
            ):
                label = TextNode(str(point))
                label.setText(str(point))
                label.setTextColor(*((.12,.18,.23,1) if args.white_background else (.85,.9,1,1)))
                node = self.render.attachNewNode(label)
                node.setScale(.43)
                node.setPos(*CONTROL_POINTS[point],.35)
                node.setBillboardPointEye()
            self.status = OnscreenText(pos=(-1.36,.91),scale=.045,
                                       align=TextNode.ALeft,mayChange=True,
                                       fg=(.12,.18,.23,1) if args.white_background else (.9,.95,1,1))
            self.accept("space",self.step)
            self.accept("a",self.toggle_mode)
            self.accept("r",self.reset)
            self.accept("u",self.change_speed,[True])
            self.accept("d",self.change_speed,[False])
            self.accept("t",self.camera_view,["top"])
            self.accept("i",self.camera_view,["iso"])
            self.accept("escape",self.userExit)
            self.camera_view("iso")
            self.draw(0)
            self.taskMgr.add(self.update,"simulation")

        def camera_view(self, view):
            self.view = view
            lens = OrthographicLens()
            aspect = self.getAspectRatio()
            width = (EXTENT + max(2.0, EXTENT*.2)) * max(1, aspect)
            lens.setFilmSize(width,width/aspect)
            lens.setNearFar(.1,250)
            self.cam.node().setLens(lens)
            if view == "top":
                self.camera.setPos(EXTENT/2,EXTENT/2,60)
                self.camera.setHpr(0,-90,0)
            else:
                self.camera.setPos(EXTENT*1.5,-EXTENT,EXTENT*1.7)
                self.camera.lookAt(EXTENT/2,EXTENT/2,0)

        def step(self):
            if self.interactive and isfinite(self.model.tNext):
                while isfinite(self.model.tNext):
                    _, execution_type = self.engine.pick_next_execution_time_type()
                    self.engine.execute_one_step()
                    if execution_type == ExecutionType.DEVS:
                        break
                self.finished = not isfinite(self.model.tNext)
                self.draw(self.engine.tNow)

        def toggle_mode(self):
            if not self.interactive:
                while (isfinite(self.model.tNext) and
                       self.engine.pick_next_execution_time_type()[0] <= self.display_time):
                    self.engine.execute_one_step()
            self.interactive = not self.interactive
            self.base_time = self.engine.tNow
            self.clock_origin = None
            self.draw(self.engine.tNow)

        def reset(self):
            self.engine.reset()
            self.finished = False
            self.base_time = 0.0
            self.clock_origin = None
            self.draw(0.0)

        def change_speed(self, increase):
            if increase:
                self.speed = self.speed*2 if self.speed <= 100 else self.speed
            else:
                self.speed = 100.0 if self.speed > 100 else max(.1, self.speed/2)
            self.base_time = self.display_time
            self.clock_origin = None
            interval = float("inf") if self.speed > 100 else self.sample_interval
            self.engine.TimeIntervalAnimation = interval
            self.engine._tNextAnimation = self.engine.tNow+interval
            self.draw(self.display_time)

        def draw(self, time):
            self.display_time = time
            positions = {}
            robot_lines = []
            for robot, sensor in zip(self.robots, self.sensors):
                position = robot.position_at(time)
                positions[robot.name] = position
                direction = robot.direction_at(time)
                if not robot._variable.direction_initialized:
                    first,second = (CONTROL_POINTS[p] for p in self.routes[robot.name][:2])
                    direction = (second[0]-first[0],second[1]-first[1])
                heading = degrees(atan2(direction[1],direction[0]))
                self.bodies[robot.name].setPos(*position,0)
                self.bodies[robot.name].setH(heading)
                chassis = self.bodies[robot.name].find("**/rounded chassis")
                chassis.setColor(*((1,.08,.06,1) if robot._variable.collision_active
                                   else (.18,.72,.30,1)))
                self.racks[robot.name].setPos(*position,0)
                robot_lines.append(
                    f"{robot.name} {robot.phase} | {sensor.name} {sensor.phase} | "
                    f"({position[0]:.3f}, {position[1]:.3f}) m | {heading:.1f} deg"
                )
            for point,node in self.points.items():
                coordinate = CONTROL_POINTS[point]
                covered = any(
                    all(abs(p-c) <= side/2 for p,c,side in
                        zip(positions[robot.name],coordinate,robot.rack_size))
                    for robot in self.robots
                )
                node.setColor(*((1,.25,.15,1) if covered else (.65,.72,.77,1)))
            speed = "Events only" if self.speed > 100 else f"{self.speed:g}x"
            mode = "Space = next DEVS step" if self.interactive else "Automatic"
            done = " | FINISHED" if self.finished else ""
            self.status.setText(
                f"{mode} | Speed {speed}\n"
                "Space = Next event | A = Manual/Auto | R = Reset\n"
                "U / D = Speed up / down\n"
                "T = Top | I = Iso | Esc = Close\n"
                "Translucent regions = full assigned path sweeps (static)\n"
                f"t = {time:.4f} s{done} | Rack {'OFF' if args.no_rack else 'ON'}\n"
                + "\n".join(robot_lines) + "\n"
                + f"Collision check {'ON' if args.collision_check else 'OFF'} | "
                  f"Stop {'OFF' if args.no_stop_on_collision else 'ON'} | "
                  f"Contacts {len(self.model.collision_history)}\n"
                + ("Red = R1 | Green = R2 | Blue = R3 | No traffic control"
                   if len(self.robots) > 1 else
                   "Red = commanded path and sweep | Red points = occupied")
            )

        def update(self, task):
            if not self.interactive and not self.finished:
                if self.clock_origin is None:
                    self.clock_origin = task.time
                target = self.base_time+(task.time-self.clock_origin)*self.speed
                if self.speed > 100:
                    # Finite test: process DEVS directly, without animation samples.
                    while isfinite(self.model.tNext):
                        self.engine.execute_one_step()
                else:
                    while (isfinite(self.model.tNext) and
                           self.engine.pick_next_execution_time_type()[0] <= target):
                        self.engine.execute_one_step()
                self.finished = not isfinite(self.model.tNext)
                self.draw(self.engine.tNow if self.finished else target)
            elif self.interactive:
                self.finished = not isfinite(self.model.tNext)
            return task.cont

    app = Viewer()
    try:
        app.run()
    finally:
        app.destroy()


if __name__ == "__main__":
    main()
