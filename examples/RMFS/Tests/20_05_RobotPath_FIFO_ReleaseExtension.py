"""Twenty robots: safe-prefix approach and actual-clearance FIFO extension."""
from importlib import import_module
from math import atan2, degrees
from .incremental_routing import generate, create_experiment_model

DATA = generate(size=10,robot_count=20,seed=20261119)
PATHS,GRID = DATA['routes'],DATA['grid']
HEADINGS = {name:degrees(atan2(route[1][1]-route[0][1],route[1][0]-route[0][0]))
            for name,route in PATHS.items()}

def create_model(**options):
    return create_experiment_model(size=10,robot_count=20,seed=20261119,
        merge_sweeps=True,event_release=True,safe_prefix=True,**options)

if __name__ == '__main__':
    print('20 robots / 10x10 grid. FIFO, actual release only; no lookahead.')
    import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
        name='20_05_RobotPath_FIFO_ReleaseExtension',no_rack=True,
        model_factory=create_model,robot_paths=PATHS,grid=GRID,initial_headings=HEADINGS,
        collision_check=True,stop_on_collision=True,render_fps=10,
        hide_robot_names=True,hide_motion_labels=True)
