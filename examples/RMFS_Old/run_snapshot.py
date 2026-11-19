"""Run and replay alternative decisions on the RMFS warehouse layout."""

from argparse import ArgumentParser

from panda3d.core import loadPrcFileData

from .playback import RouteProgress, highlight, motion_events, playback_time, register_selection_color, speed_label

from .bidirectional_experiment import STEP
from .run_panda import Viewer
from .warehouse_snapshot import OPPOSING, SCENARIOS, evaluate

A_COLOR = (0, 0, 1, 1)
B_COLOR = (1, 0, 0, 1)


class SnapshotViewer(Viewer):
    def __init__(self, scenario, speed, *, check_collisions=False):
        self.label, snapshot = scenario
        self.results = evaluate(snapshot, check_collisions=check_collisions)
        self.best = min(
            (i for i, row in enumerate(self.results) if row[2] is not None),
            key=lambda i: self.results[i][2],
        )
        self.selected = self.best
        self.speed = speed
        self.paths = []
        register_selection_color()
        super().__init__()
        for i, (_, _, _, frames) in enumerate(self.results):
            if frames:
                self.accept(str(i + 1), self.select, [i])

    def build_scene(self):
        super().build_scene()
        self.robots = (self.robot, self.robot.copyTo(self.render))
        self.loads = tuple(list(self.racks.values())[:2])
        self.loads[0].setColor(*A_COLOR, 1)
        self.loads[1].setColor(*B_COLOR, 1)

    def select(self, index):
        self.selected = index
        self.reset()

    def next_order(self):
        pass

    def reset(self):
        self.paused = True
        self.time = 0.
        frames = [
            (time, ((a, ha, wa), (b, hb, wb)))
            for time, a, ha, b, hb, wa, wb in self.results[self.selected][3]
        ]
        self.event_times = motion_events(frames)
        for path in self.paths:
            path.removeNode()
        self.paths = []
        self.visible_routes = None
        candidate = self.results[self.selected][0]
        routes = (candidate.first + candidate.onward[1:], OPPOSING)
        self.route_progress = RouteProgress(routes, frames, .75)
        self.route_colors = (A_COLOR, B_COLOR)
        self.advance(0)

    def advance(self, dt):
        candidate, status, _, frames = self.results[self.selected]
        self.time = playback_time(self.time, dt, self.speed, frames[-1][0], self.event_times)
        index = min(len(frames) - 1, int(self.time / STEP + 1e-8))
        self.update_remaining_routes(index)
        time, a, heading_a, b, heading_b, waiting_a, waiting_b = frames[index]
        colors = (A_COLOR, B_COLOR)
        waiting = (waiting_a, waiting_b)
        for robot, rack, point, heading, color, is_waiting in zip(
            self.robots, self.loads, (a, b), (heading_a, heading_b), colors, waiting
        ):
            robot.setPos(*point, 0)
            robot.setH(heading)
            rack.setPos(*point, .1)
            rack.setColor(*((0, 0, 0, 1) if is_waiting else color), 1)
            if is_waiting:
                robot.setColor(.58, .58, .58, 1, 1)
            else:
                robot.clearColor()
        state = status.upper() if index == len(frames) - 1 else "PAUSED" if self.paused else "PLAYING"
        lines = []
        for i, (option, outcome, arrival, _) in enumerate(self.results):
            marker = "*" if i == self.best else " "
            result = f"{arrival:.2f} s" if arrival is not None else outcome
            line = f"{i + 1}{marker} {option.name}\n     {result}"
            lines.append(highlight(line, i == self.selected))
        self.monitor.setText(
            f"WAREHOUSE SNAPSHOT\n{self.label.replace(', ', '\\n')}\n"
            f"{candidate.name}\n{time:.2f} s  {state}\n\n"
            + "\n".join(lines) + "\n\n* Earliest A arrival\nMixed color = shared path\nBlack rack = waiting\n1-7 Select   Space Play\nR Reset   T Top   I ISO"
            + f"\nU/D Speed   {speed_label(self.speed)}"
        )


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=int, choices=range(1, len(SCENARIOS) + 1), default=1)
    parser.add_argument("--check-collisions", action="store_true")
    args = parser.parse_args()
    loadPrcFileData("", "win-size 1320 800\nwindow-title RMFS | Snapshot route comparison")
    SnapshotViewer(SCENARIOS[args.case - 1], speed=2, check_collisions=args.check_collisions).run()


if __name__ == "__main__":
    main()
