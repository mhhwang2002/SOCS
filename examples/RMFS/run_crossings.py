"""Play the five- and ten-robot yielding experiments."""

from argparse import ArgumentParser

from panda3d.core import loadPrcFileData

from .run_panda import Viewer
from .run_three import ThreeViewer
from .playback import register_selection_color
from .warehouse_crossings import evaluate, routes_for

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
    def __init__(self, robot_count, *, check_collisions=False, crossing_delay=0.0):
        self.robot_count = robot_count
        self.label = f"{robot_count} robots at crossings"
        self.results = [
            row[:4] for row in evaluate(robot_count, check_collisions=check_collisions,
                                       crossing_delay=crossing_delay)
        ]
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
        self.build_status_labels()

    def route_display(self, option):
        routes, colors = super().route_display(option)
        extra_routes = routes_for(self.robot_count)
        return routes + extra_routes, colors + CROSSING_COLORS[:len(extra_routes)]

    def advance(self, elapsed):
        super().advance(elapsed)
        self.monitor.setText(self.monitor.getText().replace("THREE ROBOTS", "YIELDING EXPERIMENT", 1))


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, choices=(5, 10), required=True)
    parser.add_argument("--check-collisions", action="store_true",
                        help="check robot footprints at each simulation sample")
    parser.add_argument("--crossing-delay", type=float, default=0.0,
                        help="delay crossing robot entries by this many seconds")
    args = parser.parse_args()
    loadPrcFileData("", f"win-size 1320 800\nwindow-title RMFS | {args.robots} robots at crossings")
    CrossingsViewer(args.robots, check_collisions=args.check_collisions,
                    crossing_delay=args.crossing_delay).run()


if __name__ == "__main__":
    main()
