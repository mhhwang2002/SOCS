"""Manual three-robot FIFO inspection using actual clearance reports only.

Space advances one DEVS event; A toggles automatic playback, P pauses, R resets.
The unchanged shared viewer retains white background, Iso/Top and zoom/pan.
Predictive local passage lookahead is not used by this example. The historical
module name is retained so existing launch commands keep working.
"""

from importlib import import_module
import sys

from ..controlled import create_controlled_model


# Coordinates are metres. Nonuniform spacing is intentional; no new planning
# nodes or diagonal routes are introduced by the passage extension.
PATHS = {'R1': ((8,4),(8,7)),
         'R2': ((0,4),(4,4),(8,4),(10,4)),
         'R3': ((6,0),(6,8))}
XS, YS = (0,4,6,8,10), (0,1,4,7,8)
POINTS = {(x,y):(float(x),float(y)) for x in XS for y in YS}
GRID = dict(control_points=POINTS, extent=10.,
    segments=tuple(((a,y),(b,y)) for a,b in zip(XS,XS[1:]) for y in YS)+
             tuple(((x,a),(x,b)) for x in XS for a,b in zip(YS,YS[1:])))


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=True,
                 initial_headings=None):
    return create_controlled_model(PATHS,grid=GRID,lift_up=lift_up,
        collision_check=collision_check,stop_on_collision=stop_on_collision,
        initial_headings=initial_headings,event_release=True,merge_sweeps=True,
        safe_prefix=True)


def main():
    if '--baseline' in sys.argv:  # Compatibility: clearance-only is now the sole CLI mode.
        sys.argv.remove('--baseline')
    def build(**options):
        return create_model(**options)
    print('FIFO; sweep merging ON; actual clearance reports only; lookahead OFF')
    print('All three start; R3 approaches (6,1), then waits for R2 clearance. Space = next event.')
    import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
        name='03_08_RobotPath_FIFO_ClearanceOnly',
        model_factory=build,robot_paths=PATHS,grid=GRID,
        no_rack=True,collision_check=True,stop_on_collision=True,render_fps=10)


if __name__ == '__main__':
    main()
