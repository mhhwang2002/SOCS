"""Replay the fixed RMFS collision experiments in Panda3D."""

from argparse import ArgumentParser
from math import isfinite

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import ClockObject, LineSegs, OrthographicLens, TextNode, loadPrcFileData
from socs.devs.TESEngin import TESEnginBase

from .fleet import LOWER, LIFT, PICK, ScheduledOrder, make_fleet_model, mission_pose
from .model import Assignment, RobotStatus
from .motion import MissionMotion
from .run_panda import box
from .segment_experiment import scenarios


def scenario_orders(case):
    _, first, second, delay = case
    if delay is None:
        delay = first.profile.duration if first.segment == second.segment else 0.0
    orders = []
    for index, (move, start) in enumerate(zip((first, second), (0.0, delay)), 1):
        motion = MissionMotion.for_loaded_leg(move.start, move.end)
        finish = start + LIFT + motion.travel_times["TravelToWorkstation"] + PICK + LOWER
        assignment = Assignment((0, 0, index - 1), f"W{index:02}")
        robot = RobotStatus(f"MiR600-{index:02}", move.start)
        orders.append(ScheduledOrder(index, assignment, robot, motion, 0, start, finish))
    return orders


from .playback import bind_speed, speed_label


class CollisionViewer(ShowBase):
    def __init__(self, case, *, check_collisions=True):
        self.speed = 1.
        self.check_collisions = check_collisions
        super().__init__()
        self.disableMouse()
        self.setBackgroundColor(.07, .12, .17)
        clock = ClockObject.getGlobalClock()
        clock.setMode(ClockObject.MLimited)
        clock.setFrameRate(60)
        lens = OrthographicLens()
        lens.setFilmSize(15, 11)
        self.cam.node().setLens(lens)
        self.camera.setPos(0, 0, 20)
        self.camera.setHpr(0, -90, 0)
        self.label, first, second, _ = case
        self.moves = (first, second)
        self.orders = scenario_orders(case)
        self.racks = []
        self.paths = []
        for index, (move, color) in enumerate(zip(self.moves, ((.25, .72, 1, 1), (1, .68, .23, 1))), 1):
            line = LineSegs(f"Path {index}")
            line.setColor(*color)
            line.setThickness(3)
            line.moveTo(*move.start, 0)
            line.drawTo(*move.end, 0)
            self.paths.append(self.render.attachNewNode(line.create()))
            rack = self.render.attachNewNode(f"Rack {index}")
            box(rack, f"Rack {index} footprint", (0, 0, .1), (1.5, 1.5, .2), color)
            self.racks.append(rack)
        self.status = OnscreenText(parent=self.a2dTopLeft, pos=(.07, -.09), scale=.055,
                                   align=TextNode.ALeft, fg=(1, 1, 1, 1), mayChange=True)
        bind_speed(self, self.draw_frame)
        self.accept("space", self.toggle_pause)
        self.accept("r", self.reset)
        self.accept("escape", self.userExit)
        self.reset()
        self.taskMgr.add(self.tick, "collision-replay")

    def reset(self):
        self.model = make_fleet_model(self.orders)
        self.engine = TESEnginBase(self.model)
        self.engine.bVerbose = False
        self.elapsed = 0.0
        self.paused = True
        self.draw_frame()

    def toggle_pause(self):
        self.paused = not self.paused
        self.draw_frame()

    def draw_frame(self):
        phases = []
        for rack, mission, path, move in zip(self.racks, self.model.children, self.paths, self.moves):
            point, _, _ = mission_pose(mission, self.elapsed)
            rack.setPos(*point, 0)
            path.hide() if point == move.end else path.show()
            phases.append(f"{mission.robot.name}  {mission.robot.phase}")
        state = "PAUSED" if self.paused else "RUNNING"
        if not isfinite(self.engine.pick_next_execution_time_type()[0]):
            state = "COLLISION" if self.model.blocked_robots else "COMPLETE"
        self.status.setText(
            f"{self.label}\n{self.elapsed:.2f} s   {state}\n"
            + "\n".join(phases) + "\nSpace: play/pause   R: reset   Esc: exit"
            + f"\nU/D Speed   {speed_label(self.speed)}"
        )

    def tick(self, task):
        if not self.paused and isfinite(self.engine.pick_next_execution_time_type()[0]):
            dt = ClockObject.getGlobalClock().getDt()
            self.elapsed = (self.engine.pick_next_execution_time_type()[0]
                            if self.speed == float("inf") else self.elapsed + min(dt, .1) * self.speed)
            while self.engine.pick_next_execution_time_type()[0] <= self.elapsed:
                self.engine.execute_one_step()
            if self.check_collisions:
                self.model.check_collisions(self.elapsed)
            self.draw_frame()
        return task.cont


def main():
    cases = scenarios()
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=int, choices=range(1, len(cases) + 1), default=1)
    parser.add_argument("--list", action="store_true", help="list the available scenarios")
    from argparse import BooleanOptionalAction
    parser.add_argument("--check-collisions", action=BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if args.list:
        for number, case in enumerate(cases, 1):
            print(f"{number}. {case[0]}")
        return
    loadPrcFileData("", "win-size 1050 770\nwindow-title RMFS collision experiment")
    CollisionViewer(cases[args.case - 1], check_collisions=args.check_collisions).run()


if __name__ == "__main__":
    main()
