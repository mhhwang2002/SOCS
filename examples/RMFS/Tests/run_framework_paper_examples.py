"""Paper's controlled cases: --case 2 or --case 3, plus shared Viewer options.

Both cases enable command merging and clearance-driven departure. Collision
checking and stopping are ON by default. Use --headless for numerical runs.
The original geometry demonstration files remain unchanged.
"""

import argparse
from importlib import import_module
import sys

from ..controlled import create_controlled_model


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--case',choices=('2','3'),default='2')
    args,viewer_args = parser.parse_known_args()
    example = ('02_10_RobotPath_AlternatingProgress' if args.case=='2'
               else '03_04_RobotPath_SweepInterference')
    source = import_module('examples.RMFS.Tests.'+example)
    headings = getattr(source,'INITIAL_HEADINGS',None)

    def create_model(**options):
        return create_controlled_model(source.PATHS,merge_sweeps=True,
                                       event_release=True,**options)

    sys.argv = [sys.argv[0],*viewer_args]
    viewer = import_module('examples.RMFS.Tests.01_01_RobotPath_Straight')
    print(f'Paper case {args.case}: {example}; merging ON; event release ON')
    viewer.main(name=f'Paper_Controlled_{example}',no_rack=True,
                model_factory=create_model,robot_paths=source.PATHS,
                initial_headings=headings,collision_check=True,stop_on_collision=True)


if __name__ == '__main__':
    main()
