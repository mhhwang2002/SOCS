"""Display ten occupied home bays on the warehouse perimeter."""

from panda3d.core import loadPrcFileData

from .motion import heading_between
from .run_panda import Viewer, box, draw_route
from .warehouse_homes import HOME_BAYS

HOME_COLOR = (.31, .84, .91, 1)


def preview_bays():
    south = [bay for bay in HOME_BAYS if bay[1][1] < -16]
    west = [bay for bay in HOME_BAYS if bay[1][0] < 0]
    east = [bay for bay in HOME_BAYS if bay[1][0] > 42]
    north = [bay for bay in HOME_BAYS if bay[1][1] > 15]
    for bays, count in ((south, 4), (west, 2), (east, 2), (north, 2)):
        for index in range(count):
            yield bays[round(index * (len(bays) - 1) / (count - 1))]


class HomeViewer(Viewer):
    def build_scene(self):
        super().build_scene()
        box(self.render, "Perimeter floor", (21, .5, -.20), (52, 44, .25), (.21, .29, .32, 1))
        for number, (anchor, home) in enumerate(preview_bays(), 1):
            draw_route(self.render, "Home approach", (anchor, home), color=HOME_COLOR)
            box(self.render, f"Home {number:02d}", (*home, .01), (1.65, 1.65, .02), HOME_COLOR)
            robot = self.robot if number == 1 else self.robot.copyTo(self.render)
            robot.setPos(*home, 0)
            robot.setH(heading_between(home, anchor))

    def zoom_view(self, factor):
        super().zoom_view(factor)
        if self.view_mode == "top":
            aspect = .70 * self.getAspectRatio()
            width = max(57, 48 * aspect) / self.view_zoom
            self.top_lens.setFilmSize(width, width / aspect)
            self.apply_view_pan()

    def reset(self):
        self.paused = True
        self.advance(0.)

    def next_order(self):
        pass

    def toggle_pause(self):
        pass

    def advance(self, elapsed):
        self.monitor.setText(
            "HOME POSITIONS\n\n10 robots / 10 homes\n"
            "South 4   West 2\nEast 2     North 2\n\n"
            "Cyan = bay and approach\n\nT Top   I ISO\nArrows Pan   +/- Zoom"
        )


def main():
    loadPrcFileData("", "win-size 1320 800\nwindow-title RMFS | Ten home positions")
    HomeViewer().run()


if __name__ == "__main__":
    main()
