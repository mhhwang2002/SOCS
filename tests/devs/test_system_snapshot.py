from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVS import DEVS
from socs.devs.SimEngine import SimEngineThread
from socs.devs.Snapshot import SystemSnapshot


class SnapshotTestAtomic(DEVS):
    """Minimal atomic model used to test system snapshots."""

    def __init__(self, name, parent=None, delay=5):
        super().__init__(name, parent)
        self.value = 0
        self.delay = delay

    def delta_0(self):
        self.value = 0

    def tau(self):
        return self.delay

    def delta_x(self, x):
        self.value += x
        return True

    def delta_y(self):
        self.value += 1
        return self.value

    def get_state(self):
        return self.value

    def set_state(self, state):
        self.value = state


def test_engine_snapshot_restores_time_and_atomic_model_state():
    model = SnapshotTestAtomic("atomic", delay=5)
    engine = SimEngineThread(model)

    snapshot = engine.capture_snapshot()
    assert isinstance(snapshot, SystemSnapshot)
    assert snapshot.t_now == 0.0
    assert model.get_state() == 0
    assert model.timeLast() == 0.0
    assert model.timeNext() == 5.0

    model.set_state(42)
    model.set_timeLast(2.0)
    model.set_timeNext(8.0)
    engine.tNow = 2.0

    engine.restore_snapshot(snapshot)

    assert engine.tNow == 0.0
    assert model.get_state() == 0
    assert model.timeLast() == 0.0
    assert model.timeNext() == 5.0


def test_engine_snapshot_restores_nested_models_and_imminent_set():
    root = CoupledDEVS("root")
    first = SnapshotTestAtomic("first", delay=5)
    second = SnapshotTestAtomic("second", delay=5)
    root.add_child(first)
    root.add_child(second)
    engine = SimEngineThread(root)

    snapshot = engine.capture_snapshot()
    assert root.IMM == {first, second}

    first.set_state(10)
    first.set_timeLast(1.0)
    first.set_timeNext(3.0)
    second.set_state(20)
    second.set_timeLast(2.0)
    second.set_timeNext(7.0)
    root.update_tN()
    engine.tNow = 2.0

    engine.restore_snapshot(snapshot)

    assert engine.tNow == 0.0
    assert first.get_state() == 0
    assert second.get_state() == 0
    assert first.timeLast() == second.timeLast() == 0.0
    assert first.timeNext() == second.timeNext() == 5.0
    assert root.timeNext() == 5.0
    assert root.IMM == {first, second}


def test_snapshot_operations_require_paused_engine():
    model = SnapshotTestAtomic("atomic")
    engine = SimEngineThread(model)
    engine.SkipLoop = False

    try:
        engine.capture_snapshot()
    except RuntimeError as exc:
        assert "paused" in str(exc)
    else:
        raise AssertionError("Expected capture_snapshot() to reject a running engine.")
