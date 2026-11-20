"""Ten A* routes admitted incrementally by pair and cycle checks."""
from .incremental_routing import generate,create_experiment_model,run_example
DATA=generate(robot_count=10)
GRID,PATHS=DATA["grid"],DATA["routes"]

def create_model(**options):
    return create_experiment_model(robot_count=10,**options)

if __name__=="__main__":
    run_example(10)
