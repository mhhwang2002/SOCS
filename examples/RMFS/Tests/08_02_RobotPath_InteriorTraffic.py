"""Eight frogs entering and crossing the interior of the 4x4 grid."""
from ..interior_traffic import create_interior_model, make_interior_traffic, run_interior
GRID, PATHS, INITIAL_HEADINGS = make_interior_traffic(4)

def create_model(**options):
    return create_interior_model(4, **options)

if __name__ == "__main__":
    run_interior(4)
