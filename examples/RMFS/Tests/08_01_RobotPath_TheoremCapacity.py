"""Eight moving frogs occupy all 16 vertices as distinct starts and goals.

Every route is a single grid edge. Full sweeps are mutually disjoint with
1.5 m footprints on the existing 2 m grid. This is a capacity witness for
the strong safe-start/safe-goal sufficient condition, not a traffic challenge.
"""

from importlib import import_module

from ..controlled import create_controlled_model


PATHS = {
    f"R{2 * y + pair + 1}": ((2 * pair, y), (2 * pair + 1, y))
    for y in range(4)
    for pair in range(2)
}
INITIAL_HEADINGS = {name: 0 for name in PATHS}


def create_model(**options):
    options.setdefault("initial_headings", INITIAL_HEADINGS)
    return create_controlled_model(PATHS, name="08_01_TheoremCapacity", **options)


if __name__ == "__main__":
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name="08_01_TheoremCapacity", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        initial_headings=INITIAL_HEADINGS, collision_check=True, stop_on_collision=True)
