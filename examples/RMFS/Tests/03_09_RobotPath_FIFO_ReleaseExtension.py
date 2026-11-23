"""Actual clearance resumes acceleration during braking; no status request or lookahead."""
from importlib import import_module
from ..controlled import create_controlled_model

PATHS = {'R1':((20,-5),(20,5)),
         'R2':((0,0),(16,0),(20,0),(24,0)),
         'R3':((18,-8),(18,8))}
XS,YS = (0,16,18,20,24),(-8,-5,0,5,8)
GRID = dict(control_points={(x,y):(float(x),float(y)) for x in XS for y in YS},extent=24.,
    segments=tuple(((a,y),(b,y)) for a,b in zip(XS,XS[1:]) for y in YS)+
             tuple(((x,a),(x,b)) for x in XS for a,b in zip(YS,YS[1:])))

def create_model(*,lift_up=False,collision_check=True,stop_on_collision=True,initial_headings=None):
    return create_controlled_model(PATHS,grid=GRID,lift_up=lift_up,
        collision_check=collision_check,stop_on_collision=stop_on_collision,
        initial_headings=initial_headings,event_release=True,merge_sweeps=True,safe_prefix=True)

if __name__ == '__main__':
    print('FIFO, actual release only. Watch R2 brake, then accelerate after R1 clearance.')
    import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
        name='03_09_RobotPath_FIFO_ReleaseExtension',no_rack=True,model_factory=create_model,
        robot_paths=PATHS,grid=GRID,collision_check=True,stop_on_collision=True,render_fps=10)
