"""Animate a recorded run of the bidirectional aisle experiment."""

from argparse import ArgumentParser

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import ClockObject, OrthographicLens, TextNode, loadPrcFileData

from .bidirectional_experiment import BAY, CASES, EAST_EXIT, RACK_HALF, STEP, WEST_EXIT, run
from .run_panda import box
from .playback import bind_speed, playback_time, speed_label


class AisleViewer(ShowBase):
    def __init__(self, case, *, uncontrolled=False, force_exclusive=False, check_collisions=False):
        self.speed = 1.
        super().__init__()
        self.disableMouse()
        self.setBackgroundColor(.07, .12, .17)
        clock = ClockObject.getGlobalClock()
        clock.setMode(ClockObject.MLimited)
        clock.setFrameRate(60)
        lens = OrthographicLens()
        lens.setFilmSize(21, 13)
        self.cam.node().setLens(lens)
        self.camera.setPos(0, 0, 20)
        self.camera.setHpr(0, -90, 0)
        self.frames = []
        self.policy, self.result, _, self.events = run(
            case, uncontrolled=uncontrolled, force_exclusive=force_exclusive,
            record=lambda *frame: self.frames.append(frame), check_collisions=check_collisions,
        )
        self.case = case
        road = (.23, .32, .38, 1)
        box(self.render, "Main aisle", (0, 0, -.04), (18, 1.9, .08), road)
        box(self.render, "East waiting area", (6, -1.3, -.04), (1.9, 4.5, .08), road)
        if case.bay_available:
            box(self.render, "Yield loop", (1.25, 2.5, -.04), (4.4, 1.9, .08), road)
            for x in (0., 2.5):
                box(self.render, "Bay connector", (x, 1.25, -.04), (1.9, 2.5, .08), road)
        self.racks = []
        for name, color in (("A", (.2, .72, 1, 1)), ("B", (1, .63, .18, 1))):
            rack = self.render.attachNewNode(name)
            box(rack, name, (0, 0, .1), (2 * RACK_HALF, 2 * RACK_HALF, .2), color)
            self.racks.append(rack)
        for point in case.blocked:
            box(self.render, "Blocked area", (*point, .1), (1.5, 1.5, .2), (.7, .23, .36, 1))
        if "bay" in case.reserved:
            box(self.render, "Reserved bay", (*BAY, .05), (1.5, 1.5, .1), (.7, .23, .36, 1))
        self.status = OnscreenText(parent=self.a2dTopLeft, pos=(.07, -.10), scale=.055,
                                   align=TextNode.ALeft, fg=(1, 1, 1, 1), mayChange=True)
        self.event_times = sorted({when for when, _ in self.events} | {self.frames[-1][0]})
        bind_speed(self, self.draw_frame)
        self.accept("space", self.toggle)
        self.accept("r", self.reset)
        self.accept("escape", self.userExit)
        self.reset()
        self.taskMgr.add(self.tick, "bidirectional animation")

    def reset(self):
        self.elapsed = 0.
        self.paused = True
        for rack in self.racks:
            rack.clearColor()
        self.draw_frame()

    def toggle(self):
        self.paused = not self.paused
        self.draw_frame()

    def draw_frame(self):
        index = min(len(self.frames) - 1, int(self.elapsed / STEP + 1e-8))
        time, a, b = self.frames[index]
        previous = self.frames[max(0, index - 1)][1:]
        for rack, point, old_point, goal in zip(self.racks, (a, b), previous, (EAST_EXIT, WEST_EXIT)):
            rack.setPos(*point, 0)
            if point == old_point and point != goal:
                rack.setColor(0, 0, 0, 1, 1)
            else:
                rack.clearColor()
        message = next((message for when, message in reversed(self.events) if when <= time), "")
        if index == len(self.frames) - 1:
            message = self.result.upper()
            if self.result == "collision":
                for rack in self.racks:
                    rack.setColor(1, 0, 0, 1, 1)
        self.status.setText(
            f"{self.case.name}  |  {self.policy}\n"
            f"{time:.2f} s   {message}\n"
            "A = blue   B = orange\nSpace: play/pause   R: reset   Esc: exit"
            + f"\nU/D Speed   {speed_label(self.speed)}"
        )

    def tick(self, task):
        if not self.paused and self.elapsed < self.frames[-1][0]:
            self.elapsed = playback_time(self.elapsed, ClockObject.getGlobalClock().getDt(),
                                         self.speed, self.frames[-1][0], self.event_times)
            self.draw_frame()
        return task.cont


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=int, choices=range(1, len(CASES) + 1), default=1)
    parser.add_argument("--list", action="store_true")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--exclusive", action="store_true")
    mode.add_argument("--uncontrolled", action="store_true")
    parser.add_argument("--check-collisions", action="store_true")
    args = parser.parse_args()
    if args.list:
        for number, case in enumerate(CASES, 1):
            print(f"{number}. {case.name}")
        return
    loadPrcFileData("", "win-size 1100 700\nwindow-title RMFS | Bidirectional aisle")
    AisleViewer(CASES[args.case - 1], uncontrolled=args.uncontrolled,
                force_exclusive=args.exclusive,
                check_collisions=args.check_collisions or args.uncontrolled).run()


if __name__ == "__main__":
    main()
