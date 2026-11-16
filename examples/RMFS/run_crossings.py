"""Play the five- and ten-robot yielding experiments."""

from argparse import ArgumentParser

from panda3d.core import loadPrcFileData

from .run_panda import Viewer
from .run_three import WAITING, ThreeViewer
from .playback import register_selection_color
from .warehouse_crossings import STEP, evaluate, routes_for

CROSSING_COLORS = (
    (.95, .68, .23, 1),  # amber
    (.70, .48, .96, 1),  # violet
    (.20, .86, .77, 1),  # turquoise
    (.98, .51, .77, 1),  # pink
    (.82, .90, .31, 1),  # lime
    (.53, .62, 1., 1),   # periwinkle
    (1., .93, .65, 1),   # cream
)


class CrossingsViewer(ThreeViewer):
    def __init__(self, robot_count, *, check_collisions=False):
        self.robot_count = robot_count
        self.label = f"{robot_count} robots at crossings"
        self.results = [row[:4] for row in evaluate(robot_count, check_collisions=check_collisions)]
        self.selected = 0
        self.paths = []
        self.speed = 2.
        register_selection_color()
        Viewer.__init__(self)
        for index in range(len(self.results)):
            self.accept(str(index + 1), self.select, [index])

    def build_scene(self):
        Viewer.build_scene(self)
        self.robots = (self.robot, *(self.robot.copyTo(self.render) for _ in range(self.robot_count - 1)))
        self.loads = tuple(list(self.racks.values())[:self.robot_count])

    def route_display(self, option):
        routes, colors = super().route_display(option)
        extra_routes = routes_for(self.robot_count)
        return routes + extra_routes, colors + CROSSING_COLORS[:len(extra_routes)]

    def advance(self, elapsed):
        super().advance(elapsed)
        frames = self.results[self.selected][3]
        _, poses = frames[min(len(frames) - 1, int(self.time / STEP + 1e-8))]
        for robot, rack, (position, heading, waiting), color in zip(
            self.robots[3:], self.loads[3:], poses[3:], CROSSING_COLORS
        ):
            robot.setPos(*position, 0)
            robot.setH(heading)
            rack.setPos(*position, .1)
            rack.setColor(*(WAITING if waiting else color), 1)
        self.monitor.setText(self.monitor.getText().replace("THREE ROBOTS", "YIELDING EXPERIMENT", 1))


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, choices=(5, 10), required=True)
    parser.add_argument("--check-collisions", action="store_true",
                        help="check robot footprints at each simulation sample")
    args = parser.parse_args()
    loadPrcFileData("", f"win-size 1320 800\nwindow-title RMFS | {args.robots} robots at crossings")
    CrossingsViewer(args.robots, check_collisions=args.check_collisions).run()


if __name__ == "__main__":
    main()
