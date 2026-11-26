"""R1 and R3 on the original straight convoy layout."""

from importlib import import_module


CONVOY = import_module('.04_02_RobotPath_StraightConvoy', __package__)
PATHS = {name: CONVOY.PATHS[name] for name in ('R1', 'R3')}
GRID = CONVOY.GRID


def create_model(**options):
    return CONVOY.create_model(paths=PATHS, **options)


if __name__ == '__main__':
    CONVOY.main(paths=PATHS, title='02_14 Straight Convoy')
