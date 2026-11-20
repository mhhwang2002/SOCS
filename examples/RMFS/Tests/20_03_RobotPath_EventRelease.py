"""Twenty robots: release reports update RoadMap and trigger safe dispatch."""
from .incremental_routing import generate, create_experiment_model, run_example

DATA = generate(robot_count=20)
GRID, PATHS = DATA['grid'], DATA['routes']


def create_model(**options):
    return create_experiment_model(robot_count=20,merge_sweeps=True,event_release=True,**options)


if __name__ == '__main__':
    run_example(20,merge_sweeps=True,event_release=True)
