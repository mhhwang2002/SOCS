"""Ten robots on five horizontal and five vertical lanes."""

from . import crossing_stop_comparison as comparison


PATHS, GRID, CROSSINGS = comparison.crossing_layout((6, 8, 10, 12, 14), 19)


def create_model(**options):
    return comparison.create_model(PATHS, GRID, CROSSINGS,
                                   plan_builder=comparison.horizontal_first_witness, **options)


if __name__ == '__main__':
    comparison.main('10_04 Twenty-five Crossings', PATHS, GRID, CROSSINGS, create_model)
