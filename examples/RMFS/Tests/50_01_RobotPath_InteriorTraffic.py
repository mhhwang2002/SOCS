"""Fifty frogs mixing directions inside the 10x10 grid."""
from ..interior_traffic import create_interior_model, make_interior_traffic, run_interior
GRID, PATHS, INITIAL_HEADINGS = make_interior_traffic(10)

def create_model(**options):
    return create_interior_model(10, **options)

if __name__ == "__main__":
    run_interior(10)
