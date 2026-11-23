"""Four robots on two horizontal and two vertical lanes."""

from . import crossing_stop_comparison as comparison


PATHS, GRID, CROSSINGS = comparison.crossing_layout((6, 8), 13)


def create_model(**options):
    return comparison.create_model(PATHS, GRID, CROSSINGS, **options)


if __name__ == '__main__':
    comparison.main('04_03 Four Crossings', PATHS, GRID, CROSSINGS, create_model)
