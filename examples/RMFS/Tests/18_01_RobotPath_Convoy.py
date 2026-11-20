"""Eighteen frogs following the 10x10 perimeter, each moving five edges."""
from ..convoy import create_convoy_model, make_convoy, run_convoy
GRID, PATHS, INITIAL_HEADINGS = make_convoy(10)

def create_model(**options):
    return create_convoy_model(10, **options)

if __name__ == "__main__":
    run_convoy(10)
