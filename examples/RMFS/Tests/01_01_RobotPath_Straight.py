"""R1 straight run: Space advances one DEVS step; T/I select the camera."""

from argparse import ArgumentParser
from colorsys import hsv_to_rgb
from math import atan2, cos, degrees, floor, isfinite, pi, sin
from time import perf_counter

from socs.devs.TESEngin import ExecutionType, TESEnginBase

from ..GridPath import CONTROL_POINTS, POINTS_PER_AXIS, SEGMENTS, SPACING_M
from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

PATH = tuple((ix, POINTS_PER_AXIS // 2) for ix in range(POINTS_PER_AXIS))
EXTENT = (POINTS_PER_AXIS - 1) * SPACING_M
COLLISION_INTERVAL = .01


def robot_color(index):
    fixed = ((1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1),
             (1, .5, 0, 1), (0, .7, 1, 1))
    return fixed[index] if index < len(fixed) else (*hsv_to_rgb((index*.618034) % 1, .85, 1), 1)


def remaining_group_text(roadmap):
    groups = roadmap.interference_groups
    independent = roadmap.independent_objects
    lines = [f"Remaining interference groups: {len(groups)} | "
             f"Independent: {len(independent)} | Edges: {len(roadmap.interference_edges)}"]
    for index,group in enumerate(groups[:4],1):
        members = ', '.join(group[:8]) + (', ...' if len(group)>8 else '')
        lines.append(f"G{index} ({len(group)}): {members}")
    if len(groups)>4:
        lines.append(f"... {len(groups)-4} more groups")
    if independent:
        lines.append("Independent: "+', '.join(independent[:8])+(' ...' if len(independent)>8 else ''))
    return '\n'.join(lines)


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
                 stop_on_collision=True, initial_headings=None, grid=None,
                 engine_factory=TESEnginBase):
    model, robots, sensors = create_scene(
        path, name, lift_up, model_factory, collision_check=collision_check,
        stop_on_collision=stop_on_collision, initial_headings=initial_headings,
    )
    engine = engine_factory(
        model, TimeIntervalAnimation=COLLISION_INTERVAL if collision_check else float("inf"),
    )
    engine.bVerbose = False
    while isfinite(model.tNext):
        engine.execute_one_step()
    if getattr(model, "planning_error", None):
        assert all(robot.phase == "Wait" for robot in robots)
        assert not model.controller.traffic_manager.command_buffer
        print(f"NO PLAN: {model.planning_error}; no plant commands emitted.")
        return
    points_map = CONTROL_POINTS if grid is None else grid["control_points"]
    routes = robot_paths or {"R1": path}
    for robot, sensor in zip(robots, sensors):
        assert robot.phase in ("Wait","Collided")
        if robot.phase == "Wait":
            assert robot._variable.current_position == points_map[routes[robot.name][-1]]
            assert sensor.phase == "Idle"
        else:
            assert sensor.phase == "Collided"
        assert not sensor._variable.pending_events
    if hasattr(model, "controller"):
        manager = model.controller.traffic_manager
        assert manager.done and not manager.awaiting and not manager.command_buffer
        assert not model.collision_history
        progress = (f"{len(manager.finished_tasks)}/{len(manager.tasks)} movement tasks"
                    if manager.task_control else f"{len(manager.completed_stages)} tuple stages")
        print(f"TrafficManager completed {progress}; "
              f"commands={manager.next_command_id}; contacts=0")
    states = "; ".join(
        f"{robot.name}={robot.phase}; {sensor.name}={sensor.phase}"
        for robot, sensor in zip(robots, sensors)
    )
    print(f"Finished at t={engine.tNow:.6f}; {states}")


def main(path=PATH, *, name="01_01_RobotPath_Straight", no_rack=False,
         model_factory=None, robot_paths=None, collision_check=False,
         stop_on_collision=True, initial_headings=None, grid=None, hide_robot_names=False,
         hide_motion_labels=False, render_fps=30, hide_point_labels=False,
         engine_factory=TESEnginBase, argv=None, toggle_passage=None):
    control_points = CONTROL_POINTS if grid is None else grid["control_points"]
    segments = SEGMENTS if grid is None else grid["segments"]
    extent = EXTENT if grid is None else grid["extent"]
    view_center = tuple((min(p[k] for p in control_points.values())+
                         max(p[k] for p in control_points.values()))/2 for k in (0,1))
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--non-interactive", action="store_true",
                        help="advance automatically to the final point")
    parser.add_argument("--headless", action="store_true",
                        help="run only DEVS without opening a window")
    parser.add_argument("--white-background", action="store_true", default=True,
                        help="white background with dark text and grid (default, for publication)")
    parser.add_argument("--no-rack", action="store_true", default=no_rack,
                        help="run an empty robot using empty motion limits")
    name_options = parser.add_mutually_exclusive_group()
    name_options.add_argument('--hide-robot-names',action='store_true')
    name_options.add_argument('--show-robot-names',dest='hide_robot_names',action='store_false')
    parser.set_defaults(hide_robot_names=hide_robot_names)
    motion_options = parser.add_mutually_exclusive_group()
    motion_options.add_argument('--hide-motion-labels',action='store_true')
    motion_options.add_argument('--show-motion-labels',dest='hide_motion_labels',action='store_false')
    parser.set_defaults(hide_motion_labels=hide_motion_labels)
    point_options = parser.add_mutually_exclusive_group()
    point_options.add_argument('--hide-point-labels',action='store_true')
    point_options.add_argument('--show-point-labels',dest='hide_point_labels',action='store_false')
    parser.set_defaults(hide_point_labels=hide_point_labels)
    sweep_options = parser.add_mutually_exclusive_group()
    sweep_options.add_argument('--hide-sweeps', action='store_true',
                               help='skip sweep geometry rendering; keep traffic control unchanged')
    sweep_options.add_argument('--show-sweeps', dest='hide_sweeps', action='store_false')
    parser.set_defaults(hide_sweeps=False)
    parser.add_argument('--fps',type=int,default=render_fps,help='viewer frames per second (independent of DEVS time)')
    parser.add_argument('--model-style', choices=('wire', 'surface'), default='wire',
                        help='rack/chassis drawing style; eyes remain solid (V toggles)')
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
    args = parser.parse_args(argv)
    if args.fps<1:
        parser.error('--fps must be positive')
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
                     initial_headings=headings, grid=grid, engine_factory=engine_factory)
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
        node = GeomNode("surface geometry")
        node.addGeom(geom)
        result = parent.attachNewNode('rounded chassis')
        surface = result.attachNewNode('surface')
        surface.attachNewNode(node)
        wire = result.attachNewNode('wire')
        lines = LineSegs('chassis outline')
        lines.setThickness(1.5)
        for z in (0., .322):
            lines.moveTo(*outline[0],z)
            for x,y in (*outline[1:],outline[0]):
                lines.drawTo(x,y,z)
        for index in (0,1,10,len(outline)-1):
            x,y = outline[index]
            lines.moveTo(x,y,0)
            lines.drawTo(x,y,.322)
        wire.attachNewNode(lines.create())
        # LineSegs installs its own white color state. Override that state,
        # otherwise the wire chassis disappears against the white background.
        result.setColor(.18,.72,.30,1,1)
        return result

    def low_poly_eye(parent):
        # Eight longitudes, four latitude bands: 48 nondegenerate triangles.
        data = GeomVertexData('eye', GeomVertexFormat.getV3(), Geom.UHStatic)
        writer = GeomVertexWriter(data, 'vertex')
        for ring in range(5):
            latitude = -pi/2 + ring*pi/4
            for segment in range(8):
                angle = segment*pi/4
                writer.addData3(cos(latitude)*cos(angle), cos(latitude)*sin(angle), sin(latitude))
        triangles = GeomTriangles(Geom.UHStatic)
        for ring in range(4):
            for segment in range(8):
                a, b = ring*8+segment, ring*8+(segment+1)%8
                if ring > 0:
                    triangles.addVertices(a, b, a+8)
                if ring < 3:
                    triangles.addVertices(b, b+8, a+8)
        geom = Geom(data)
        geom.addPrimitive(triangles)
        node = GeomNode('eye')
        node.addGeom(geom)
        return parent.attachNewNode(node)

    loadPrcFileData("", f"window-title RMFS | {name}")
    loadPrcFileData("", "win-size 1200 850")
    loadPrcFileData("", "sync-video true")

    class Viewer(ShowBase):
        def __init__(self):
            super().__init__()
            self.disableMouse()
            self.setBackgroundColor(*((1,1,1) if args.white_background else (.06,.10,.14)))
            ClockObject.getGlobalClock().setMode(ClockObject.MLimited)
            ClockObject.getGlobalClock().setFrameRate(args.fps)
            self.model, self.robots, self.sensors = create_scene(
                path, name, not args.no_rack, model_factory,
                collision_check=args.collision_check,
                stop_on_collision=not args.no_stop_on_collision,
                initial_headings=headings,
            )
            self.robot, self.sensor = self.robots[0], self.sensors[0]
            self.routes = robot_paths or {self.robot.name: path}
            self.sample_interval = COLLISION_INTERVAL if args.collision_check else float("inf")
            self.engine = engine_factory(
                self.model, TimeIntervalAnimation=self.sample_interval,
            )
            self.engine.bVerbose = False
            self.view = "iso"
            self.clock_origin = None
            self.base_time = 0.0
            self.speed = 1.0
            self.finished = False
            self.interactive = not args.non_interactive
            self.paused = False
            self.model_style = args.model_style
            self.rack = self.render.attachNewNode("Rack1")
            self.body = self.render.attachNewNode("R1")
            rounded_chassis(self.body)
            for y in (-.25,.25):
                eye = low_poly_eye(self.body)
                eye.setPos(.48,y,.38)
                eye.setScale(.24,.21,.24)
                eye.setColor(.9,1,1,1)
                pupil = low_poly_eye(self.body)
                pupil.setPos(.67,y,.42)
                pupil.setScale(.075,.115,.13)
                pupil.setColor(.025,.07,.12,1)
            rack_x, rack_y = self.robot.rack_size
            rack_surface = self.rack.attachNewNode('surface')
            box(rack_surface,(0,0,1.91),(rack_x,rack_y,.08),(1,.75,.1,1))
            for x in (-rack_x/2+.05,rack_x/2-.05):
                for y in (-rack_y/2+.05,rack_y/2-.05):
                    box(rack_surface,(x,y,1.0),(.1,.1,1.9),(1,.75,.1,1))
            for z in (.5,.84,1.18,1.52):
                # Thin shelves need just one two-sided rectangle, not six box faces.
                data = GeomVertexData('shelf', GeomVertexFormat.getV3(), Geom.UHStatic)
                writer = GeomVertexWriter(data, 'vertex')
                for x,y in ((-1,-1),(1,-1),(1,1),(-1,1)):
                    writer.addData3(x*(rack_x-.06)/2,y*(rack_y-.06)/2,z)
                triangles = GeomTriangles(Geom.UHStatic)
                triangles.addVertices(0,1,2)
                triangles.addVertices(0,2,3)
                geom = Geom(data)
                geom.addPrimitive(triangles)
                node = GeomNode('shelf')
                node.addGeom(geom)
                shelf = rack_surface.attachNewNode(node)
                shelf.setColor(1,.75,.1,1)
                shelf.setTwoSided(True)
            rack_surface.flattenStrong()
            rack_wire = self.rack.attachNewNode('wire')
            lines = LineSegs('rack outline')
            lines.setThickness(1.2)
            for x in (-rack_x/2+.05,rack_x/2-.05):
                for y in (-rack_y/2+.05,rack_y/2-.05):
                    lines.moveTo(x,y,.05)
                    lines.drawTo(x,y,1.95)
            for z in (.5,.84,1.18,1.52,1.91):
                corners = tuple((x*rack_x/2,y*rack_y/2,z)
                                for x,y in ((-1,-1),(1,-1),(1,1),(-1,1)))
                lines.moveTo(*corners[0])
                for point in (*corners[1:],corners[0]):
                    lines.drawTo(*point)
            rack_wire.attachNewNode(lines.create())
            if args.no_rack:
                self.rack.hide()
            self.bodies = {self.robot.name: self.body}
            self.racks = {self.robot.name: self.rack}
            for robot in self.robots[1:]:
                self.bodies[robot.name] = self.body.copyTo(self.render)
                self.racks[robot.name] = self.rack.copyTo(self.render)
            self.motion_labels = {}
            for robot in self.robots:
                if args.hide_motion_labels:
                    continue
                text = TextNode(f"motion-{robot.name}")
                text.setTextColor(*((.12,.18,.23,1) if args.white_background else (.9,.95,1,1)))
                node = self.bodies[robot.name].attachNewNode(text)
                node.setPos(0,0,1.35)
                node.setScale(.4)
                node.setBillboardPointEye()
                self.motion_labels[robot.name] = text
            route_colors = tuple(robot_color(i) for i in range(len(self.robots)))
            for index,robot in enumerate(self.robots):
                self.racks[robot.name].setColor(*route_colors[index],1)
            self.warehouse_racks = {}
            if grid is not None and 'warehouse_racks' in grid:
                background = self.render.attachNewNode('warehouse static racks')
                empty = set(grid.get('warehouse_empty_cells',()))
                for cell,position in grid['warehouse_racks'].items():
                    if cell in empty:
                        continue
                    rack = self.rack.copyTo(background)
                    rack.show()
                    rack.setPos(*position,0)
                    rack.setColor(.95,.72,.17,1,2)
                    self.warehouse_racks[cell] = rack
                for station,(x,y) in grid['warehouse_stations'].items():
                    box(self.render,(x,y+1.75,.45),(2.2,1.,.9),(.25,.5,.55,1))
                    label=TextNode('station-'+station)
                    label.setText(station);label.setTextColor(.08,.16,.2,1)
                    node=self.render.attachNewNode(label)
                    node.setPos(x,y+1.75,1.1);node.setScale(.6);node.setBillboardPointEye()
            if len(self.robots) > 1 and not args.hide_robot_names:
                for index, robot in enumerate(self.robots):
                    label = TextNode(robot.name)
                    label.setText(robot.name)
                    label.setTextColor(*route_colors[index % len(route_colors)])
                    node = self.bodies[robot.name].attachNewNode(label)
                    node.setPos(0,0,.9)
                    node.setScale(.45)
                    node.setBillboardPointEye()
            self.points = {}
            grid_lines = LineSegs()
            grid_lines.setColor(*((.55,.6,.63,1) if args.white_background else (.35,.43,.48,1)))
            for start,end in segments:
                grid_lines.moveTo(*control_points[start],.01)
                grid_lines.drawTo(*control_points[end],.01)
            self.render.attachNewNode(grid_lines.create())
            self.sweeps = {}
            self.show_sweeps = not args.hide_sweeps
            self.route_lines = {}
            self.sweep_progress = {}
            self.route_colors = dict(zip(self.routes,route_colors))
            robots_by_name = {robot.name: robot for robot in self.robots}
            for index,(robot_name,route_path) in enumerate(self.routes.items()):
                if not self.show_sweeps:
                    continue
                self.sweeps[robot_name] = draw_sweep(
                    self.render, tuple(control_points[p] for p in route_path),
                    robots_by_name[robot_name].rack_size,
                    route_colors[index % len(route_colors)],
                )
            for index, (robot_name,route_path) in enumerate(self.routes.items()):
                route = LineSegs()
                route.setThickness(4)
                route.setColor(*route_colors[index % len(route_colors)])
                route.moveTo(*control_points[route_path[0]],.04)
                for point in route_path[1:]:
                    route.drawTo(*control_points[point],.04)
                self.route_lines[robot_name] = self.render.attachNewNode(route.create())
            for point in control_points:
                x,y = control_points[point]
                self.points[point] = box(self.render,(x,y,.06),(.13,.13,.1),
                                         (.65,.72,.77,1))
            self.point_cell_size = max(max(robot.rack_size) for robot in self.robots)
            self.point_buckets = {}
            for point,(x,y) in control_points.items():
                cell = (floor(x/self.point_cell_size),floor(y/self.point_cell_size))
                self.point_buckets.setdefault(cell,[]).append(point)
            self.covered_points = set()
            self.chassis_nodes = {name:body.find('**/rounded chassis') for name,body in self.bodies.items()}
            self.chassis_collision = {}
            self.surface_nodes = tuple(self.render.findAllMatches('**/surface'))
            self.wire_nodes = tuple(self.render.findAllMatches('**/wire'))
            self.apply_model_style()
            for point in dict.fromkeys(
                point for route_path in self.routes.values() for point in route_path
            ):
                if args.hide_point_labels:
                    continue
                label = TextNode(str(point))
                label.setText(str(point))
                label.setTextColor(*((.12,.18,.23,1) if args.white_background else (.85,.9,1,1)))
                node = self.render.attachNewNode(label)
                node.setScale(.43)
                node.setPos(*control_points[point],.35)
                node.setBillboardPointEye()
            self.status = OnscreenText(pos=(-1.36,.91),scale=.045,
                                       align=TextNode.ALeft,mayChange=True,
                                       fg=(.12,.18,.23,1) if args.white_background else (.9,.95,1,1))
            self.time_label = OnscreenText(pos=(-1.36,.91),scale=.045,
                                       align=TextNode.ALeft,mayChange=True,
                                       fg=(.12,.18,.23,1) if args.white_background else (.9,.95,1,1))
            self.time_label.hide()
            self.zoom_factor = 1.0
            self.pan_offset = [0.,0.]
            self.accept("space",self.step)
            self.accept("a",self.toggle_mode)
            self.accept("p",self.toggle_pause)
            self.accept("v",self.toggle_model_style)
            self.accept("r",self.reset)
            if toggle_passage is not None:
                self.accept('m', self.toggle_passage_mode)
            self.accept("u",self.change_speed,[True])
            self.accept("d",self.change_speed,[False])
            self.accept("t",self.camera_view,["top"])
            self.accept("i",self.camera_view,["iso"])
            self.accept("h",self.toggle_info)
            self.accept("s",self.toggle_sweeps)
            self.accept("+",self.zoom,[1.15])
            self.accept("=",self.zoom,[1.15])
            self.accept("-",self.zoom,[1/1.15])
            self.accept("arrow_left",self.pan,[-1,0])
            self.accept("arrow_right",self.pan,[1,0])
            self.accept("arrow_up",self.pan,[0,1])
            self.accept("arrow_down",self.pan,[0,-1])
            self.accept("escape",self.userExit)
            self.camera_view("iso")
            self.draw(0)
            self.taskMgr.add(self.update,"simulation")

        def camera_view(self, view):
            self.view = view
            lens = OrthographicLens()
            aspect = self.getAspectRatio()
            width = (extent + max(2.0, extent*.2)) * max(1, aspect)
            if hasattr(self.model, "controller"):
                width *= 1.12  # keep W/R labels inside the controlled example's view
            width /= self.zoom_factor
            lens.setFilmSize(width,width/aspect)
            lens.setFilmOffset(-self.pan_offset[0],-self.pan_offset[1])
            lens.setNearFar(.1,250)
            self.cam.node().setLens(lens)
            if view == "top":
                self.camera.setPos(*view_center,max(60,extent*2))
                self.camera.setHpr(0,-90,0)
            else:
                self.camera.setPos(view_center[0]+extent,view_center[1]-extent*1.5,extent*1.7)
                self.camera.lookAt(*view_center,0)

        def apply_model_style(self):
            for node in self.surface_nodes:
                node.show() if self.model_style == 'surface' else node.hide()
            for node in self.wire_nodes:
                node.show() if self.model_style == 'wire' else node.hide()

        def toggle_model_style(self):
            self.model_style = 'surface' if self.model_style == 'wire' else 'wire'
            self.apply_model_style()
            self.draw(self.display_time)

        def toggle_pause(self):
            self.paused = not self.paused
            self.base_time = self.display_time
            self.clock_origin = None
            self.draw(self.display_time)

        def toggle_sweeps(self):
            self.show_sweeps = not self.show_sweeps
            for node in self.sweeps.values():
                node.removeNode()
            self.sweeps.clear()
            if self.show_sweeps:
                roadmap = getattr(getattr(self.model, "controller", None), "roadmap", None)
                for robot in self.robots:
                    coordinates = (roadmap.remaining_subpath(robot.name).coordinates()
                                   if roadmap is not None else
                                   tuple(control_points[p] for p in self.routes[robot.name]))
                    self.sweeps[robot.name] = draw_sweep(
                        self.render, coordinates, robot.rack_size, self.route_colors[robot.name])
            self.draw(self.display_time)

        def toggle_info(self):
            if self.status.isHidden():
                self.status.show()
                self.time_label.hide()
                self.draw(self.display_time)
            else:
                self.status.hide()
                self.time_label.show()

        def zoom(self, factor):
            self.zoom_factor = min(20.,max(.05,self.zoom_factor*factor))
            self.camera_view(self.view)

        def pan(self, dx, dy):
            size = self.cam.node().getLens().getFilmSize()
            self.pan_offset[0] += dx*size[0]*.04
            self.pan_offset[1] += dy*size[1]*.04
            self.camera_view(self.view)

        def step(self):
            if (self.interactive or self.paused) and isfinite(self.model.tNext):
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

        def toggle_passage_mode(self):
            toggle_passage()
            self.model, self.robots, self.sensors = create_scene(
                path, name, not args.no_rack, model_factory,
                collision_check=args.collision_check,
                stop_on_collision=not args.no_stop_on_collision,
                initial_headings=headings,
            )
            self.robot, self.sensor = self.robots[0], self.sensors[0]
            interval = float('inf') if self.speed > 100 else self.sample_interval
            self.engine = engine_factory(self.model, TimeIntervalAnimation=interval)
            self.engine.bVerbose = False
            self.reset()

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
            speed = "Events only" if self.speed > 100 else f"{self.speed:g}x"
            self.time_label.setText(f"t = {time:.4f} s | Speed {speed}" + (' | PAUSED' if self.paused else ''))
            roadmap = getattr(getattr(self.model,"controller",None),"roadmap",None)
            if roadmap is not None:
                for robot in self.robots:
                    name = robot.name
                    progress = (roadmap.paths[name], roadmap.robots[name]["released_progress"])
                    if self.sweep_progress.get(name) == progress:
                        continue
                    self.sweep_progress[name] = progress
                    coordinates = roadmap.remaining_subpath(name).coordinates()
                    if self.show_sweeps:
                        self.sweeps[name].removeNode()
                        self.sweeps[name] = draw_sweep(self.render,coordinates,robot.rack_size,
                                                       self.route_colors[name])
                    self.route_lines[name].removeNode()
                    line = LineSegs()
                    line.setThickness(4)
                    line.setColor(*self.route_colors[name])
                    line.moveTo(*coordinates[0],.04)
                    for point in coordinates[1:]:
                        line.drawTo(*point,.04)
                    self.route_lines[name] = self.render.attachNewNode(line.create())
            positions = {}
            robot_lines = []
            for robot, sensor in zip(self.robots, self.sensors):
                position = robot.position_at(time)
                positions[robot.name] = position
                direction = robot.direction_at(time)
                if not robot._variable.direction_initialized and len(self.routes[robot.name]) > 1:
                    first,second = (control_points[p] for p in self.routes[robot.name][:2])
                    direction = (second[0]-first[0],second[1]-first[1])
                heading = degrees(atan2(direction[1],direction[0]))
                self.bodies[robot.name].setPos(*position,0)
                self.bodies[robot.name].setH(heading)
                collision = robot._variable.collision_active
                if self.chassis_collision.get(robot.name) != collision:
                    normal_color = ((.18,.72,.30,1) if len(self.robots) == 1
                                    else self.route_colors[robot.name])
                    self.chassis_nodes[robot.name].setColor(
                        *((1,.08,.06,1) if collision else normal_color), 1)
                    self.chassis_collision[robot.name] = collision
                self.racks[robot.name].setPos(*position,0)
                if robot._variable.lift_up:
                    self.racks[robot.name].show()
                else:
                    self.racks[robot.name].hide()
                if robot.name in self.motion_labels:
                    self.motion_labels[robot.name].setText(
                        "W" if robot.phase == "Wait" else "R" if robot.phase == "RotateTo" else "")
                robot_lines.append(
                    f"{robot.name} {robot.phase} | {sensor.name} {sensor.phase} | "
                    f"({position[0]:.3f}, {position[1]:.3f}) m | {heading:.1f} deg"
                )
            carried = {robot._variable.rack for robot in self.robots if robot._variable.lift_up}
            for cell, rack in self.warehouse_racks.items():
                if cell in carried:
                    rack.hide()
                else:
                    rack.show()
            covered = set()
            cell_size = self.point_cell_size
            for robot in self.robots:
                x,y = positions[robot.name]
                hx,hy = (side/2 for side in robot.rack_size)
                for ix in range(floor((x-hx)/cell_size),floor((x+hx)/cell_size)+1):
                    for iy in range(floor((y-hy)/cell_size),floor((y+hy)/cell_size)+1):
                        for point in self.point_buckets.get((ix,iy),()):
                            px,py = control_points[point]
                            if abs(x-px)<=hx and abs(y-py)<=hy:
                                covered.add(point)
            for point in covered ^ self.covered_points:
                self.points[point].setColor(*((1,.25,.15,1) if point in covered else (.65,.72,.77,1)))
            self.covered_points = covered
            if self.status.isHidden():
                return
            speed = "Events only" if self.speed > 100 else f"{self.speed:g}x"
            mode = "Space = next DEVS step" if self.interactive else "Automatic"
            done = " | FINISHED" if self.finished else ""
            traffic = "No traffic control"
            if hasattr(self.model, "controller"):
                manager = self.model.controller.traffic_manager
                stage = min(manager.stage + 1, len(manager.tuple_path.stages))
                awaiting = (', '.join(sorted(manager.awaiting)) or '-'
                            if len(manager.awaiting) <= 5 else f'{len(manager.awaiting)} robots')
                traffic = (f"TrafficManager stage {stage}/{len(manager.tuple_path.stages)} | "
                           f"Buffer: {len(manager.command_buffer)}\n"
                           f"Awaiting: {awaiting} | "
                           f"{'DONE' if manager.done else 'HALTED' if manager.halted else 'RUNNING'}")
                if manager.event_release:
                    releases = sum(e['event']=='release_dispatch' for e in roadmap.history)
                    traffic += f"\nEvent release ON | Release dispatches: {releases}"
                traffic += '\n'+remaining_group_text(roadmap)
                controller = self.model.controller
                if getattr(controller, 'full_cycle', False):
                    traffic += (f"\nDeparture: {controller.departure_policy}"
                                f"\nOrders {controller.received_orders} | Shipped {controller.shipped_orders}"
                                f" | Returned {controller.completed_orders}"
                                f" | Pending {len(controller.pending_orders)}")
            if len(robot_lines) > 8:
                robot_lines = robot_lines[:8] + [f"... {len(self.robots)} robots in total"]
            if getattr(self.model, "planning_error", None):
                traffic = "NO PLAN: " + self.model.planning_error
            passage = ''
            if toggle_passage is not None:
                enabled = self.model.controller.traffic_manager.motion_extension
                passage = f"M = Push/Pull {'YES' if enabled else 'NO'} (restart)\n"
            self.status.setText(
                f"{mode} | Speed {speed} | View {args.fps} fps\n"
                "Space = Next event | A = Manual/Auto | R = Reset\n"
                f"P = Pause/resume {'[PAUSED]' if self.paused else ''} | V = Models {self.model_style}\n"
                "U / D = Speed up / down\n"
                "T = Top | I = Iso | Esc = Close\n"
                "H = Info | +/- = Zoom | Arrows = Pan\n"
                + passage
                + ("S = Sweeps ON (toggle)\n" if self.show_sweeps else "S = Sweeps OFF (toggle)\n")
                + f"t = {time:.4f} s | Speed {speed}{done} | Rack {'OFF' if args.no_rack else 'ON'}\n"
                + "\n".join(robot_lines) + "\n"
                + f"Collision check {'ON' if args.collision_check else 'OFF'} | "
                  f"Stop {'OFF' if args.no_stop_on_collision else 'ON'} | "
                  f"Contacts {len(self.model.collision_history)}\n"
                + (f"Path colors match robot labels\n{traffic}"
                   if len(self.robots) > 1 else
                   "Red = commanded path and sweep | Red points = occupied")
            )

        def update(self, task):
            if not self.interactive and not self.paused and not self.finished:
                if self.clock_origin is None:
                    self.clock_origin = task.time
                target = self.base_time+(task.time-self.clock_origin)*self.speed
                # Yield to Panda3D between event batches. This wall-clock budget
                # never adds a simulation event or changes DEVS event order.
                deadline = perf_counter()+min(.05,.6/args.fps)
                while (isfinite(self.model.tNext) and
                       (self.speed > 100 or self.engine.pick_next_execution_time_type()[0] <= target)):
                    self.engine.execute_one_step()
                    if perf_counter() >= deadline:
                        break
                self.finished = not isfinite(self.model.tNext)
                behind = (not self.finished and (self.speed > 100 or
                          self.engine.pick_next_execution_time_type()[0] <= target))
                self.draw(self.engine.tNow if self.finished or behind else target)
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
