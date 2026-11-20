"""Five frogs on the existing 4x4 grid; all five have interfering routes.

R1/R2/R3 contend on row 1; R4/R5 contend on row 3. Every start and
goal footprint is disjoint from every other full sweep (Theorem 3).
The two contention groups are not claimed to be one connected group.
Original paths are simple. Wait duplication supplies the execution tuples.
"""

from importlib import import_module

from ..controlled import create_controlled_model


PATHS = {
    "R1": ((0, 0), (0, 1), (1, 1), (2, 1), (3, 1), (3, 0)),
    "R2": ((1, 0), (1, 1), (2, 1), (2, 0)),
    "R3": ((0, 2), (0, 1), (1, 1), (2, 1), (3, 1), (3, 2)),
    "R4": ((0, 3), (1, 3), (2, 3), (3, 3)),
    "R5": ((1, 2), (1, 3), (2, 3), (2, 2)),
}
INITIAL_HEADINGS = {"R1": 90, "R2": 90, "R3": 270, "R4": 0, "R5": 90}


def create_model(**options):
    options.setdefault("initial_headings", INITIAL_HEADINGS)
    return create_controlled_model(PATHS, name="05_01_TheoremSafeEndpoints", **options)


if __name__ == "__main__":
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name="05_01_TheoremSafeEndpoints", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        initial_headings=INITIAL_HEADINGS, collision_check=True, stop_on_collision=True)
