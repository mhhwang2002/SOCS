"""Replay the three-robot warehouse experiment in Panda3D."""

from argparse import ArgumentParser
from .playback import RouteProgress, highlight, motion_events, playback_time, register_selection_color, speed_label

from panda3d.core import loadPrcFileData

from .run_panda import Viewer
from .warehouse_three import CASES, C_ROUTE, STEP, evaluate
from .warehouse_snapshot import OPPOSING

COLORS = ((0, 0, 1, 1), (1, 0, 0, 1), (0, 1, 0, 1))
WAITING = (0, 0, 0, 1)


class ThreeViewer(Viewer):
    def __init__(self, case, *, check_collisions=False):
        self.speed = 2.
        register_selection_color()
        self.label = CASES[case]
        self.results = evaluate(case, check_collisions=check_collisions)
        self.selected = 0
        self.paths = []
        super().__init__()
        for index in range(len(self.results)):
            self.accept(str(index + 1), self.select, [index])

    def build_scene(self):
        super().build_scene()
        self.robots = (self.robot, self.robot.copyTo(self.render), self.robot.copyTo(self.render))
        self.loads = tuple(list(self.racks.values())[:3])

    def next_order(self):
        pass

    def select(self, index):
        self.selected = index
        self.reset()

    def reset(self):
        self.paused = True
        self.time = 0.
        self.event_times = motion_events(self.results[self.selected][3])
        for path in self.paths:
            path.removeNode()
        self.paths = []
        self.visible_routes = None
        option = self.results[self.selected][0]
        routes, colors = self.route_display(option)
        self.route_progress = RouteProgress(routes, self.results[self.selected][3], .75)
        self.route_colors = colors
        self.advance(0.)

    def route_display(self, option):
        return (option.first + option.onward[1:], OPPOSING, C_ROUTE), COLORS

    def advance(self, elapsed):
        option, status, _, frames = self.results[self.selected]
        self.time = playback_time(self.time, elapsed, self.speed, frames[-1][0], self.event_times)
        index = min(len(frames) - 1, int(self.time / STEP + 1e-8))
        self.update_remaining_routes(index)
        time, poses = frames[index]
        for robot, rack, color, (position, heading, waiting) in zip(
            self.robots, self.loads, COLORS, poses
        ):
            robot.setPos(*position, 0)
            robot.setH(heading)
            rack.setPos(*position, .1)
            rack.setColor(*(WAITING if waiting else color), 1)
        state = status.upper() if index == len(frames) - 1 else "PAUSED" if self.paused else "PLAYING"
        lines = []
        for number, (candidate, outcome, makespan, _) in enumerate(self.results):
            marker = ">" if number == self.selected else " "
            result = f"{makespan:.2f} s" if makespan is not None else outcome
            lines.append(highlight(f"{marker}{number + 1} {candidate.name}\n     {result}", number == self.selected))
        self.monitor.setText(
            f"THREE ROBOTS\n{self.label}\n{option.name}\n{time:.2f} s  {state}\n\n"
            + "\n".join(lines) + f"\n\n1-7 Select   Space Play\nR Reset   T Top   I ISO\nU/D Speed   {speed_label(self.speed)}"
        )


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=int, choices=(1, 2), default=1)
    parser.add_argument("--check-collisions", action="store_true")
    args = parser.parse_args()
    loadPrcFileData("", "win-size 1320 800\nwindow-title RMFS | Three robots")
    ThreeViewer(args.case - 1, check_collisions=args.check_collisions).run()


if __name__ == "__main__":
    main()
