"""Six robots on three horizontal and three vertical lanes."""

from . import crossing_stop_comparison as comparison


PATHS, GRID, CROSSINGS = comparison.crossing_layout((6, 8, 10), 15)


def create_model(**options):
    return comparison.create_model(PATHS, GRID, CROSSINGS,
                                   plan_builder=comparison.horizontal_first_witness, **options)


if __name__ == '__main__':
    comparison.main('06_02 Nine Crossings', PATHS, GRID, CROSSINGS, create_model)
