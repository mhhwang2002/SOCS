"""02_10 routes executed by RMFSController / TrafficManager / RoadMap.

Safe command groups may span turns. Push/Pull extends straight movements.
Manual by default; A toggles automatic mode.
"""

from functools import partial
from importlib import import_module
import sys

from ..controlled import create_controlled_model
from ..DeadlockAvoidingTrafficManager import DeadlockAvoidingTrafficManager
from ..TrafficManager import TrafficManager

source = import_module("examples.RMFS.Tests.02_10_RobotPath_AlternatingProgress")
PATHS = source.PATHS


def create_model(*, event_release=True, safe_prefix=True, passage_push=True,
                 merge_sweeps=True, deadlock_avoidance=False, **options):
    manager = (DeadlockAvoidingTrafficManager if deadlock_avoidance
               else partial(TrafficManager, extend_after_turn=True))
    return create_controlled_model(
        PATHS, name="02_12_ControlledAlternating", event_release=event_release,
        safe_prefix=safe_prefix, passage_push=passage_push,
        merge_sweeps=merge_sweeps and not deadlock_avoidance,
        traffic_manager_type=manager, **options,
    )


if __name__ == "__main__":
    avoid_deadlock = "--deadlock-avoidance" in sys.argv
    if avoid_deadlock:
        sys.argv.remove("--deadlock-avoidance")
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name="02_12_ControlledAlternating", no_rack=True,
        model_factory=partial(create_model, deadlock_avoidance=avoid_deadlock), robot_paths=PATHS,
        collision_check=True, stop_on_collision=True)
