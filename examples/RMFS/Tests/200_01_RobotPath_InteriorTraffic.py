"""Two hundred frogs mixing directions inside the 20x20 grid."""
from ..interior_traffic import create_interior_model, make_interior_traffic, run_interior
GRID, PATHS, INITIAL_HEADINGS = make_interior_traffic(20)

def create_model(**options):
    return create_interior_model(20, **options)

if __name__ == "__main__":
    run_interior(20)
