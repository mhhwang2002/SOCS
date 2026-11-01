import pytest

from socs.devs.FDDEVS import FDDEVS
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.Snapshot import ModelSnapshot


class CounterModel(FDDEVS):
    """Simple atomic model for snapshot tests."""

    def __init__(self, name, delay=5, parent=None):
        super().__init__(name, parent)
        self.delay = delay
        self.value = 0
        self.data = {"items": [1, 2, 3]}

    def delta_0(self):
        self.value = 0
        self.data = {"items": [1, 2, 3]}

    def tau(self):
        return self.delay

    def delta_x(self, x):
        self.value += x
        return True

    def delta_y(self):
        self.value += 1
        return None

    def get_state(self):
        return {
            "value": self.value,
            "data": self.data,
        }

    def set_state(self, state):
        self.value = state["value"]
        self.data = state["data"]


@pytest.fixture
def atomic_model():
    model = CounterModel("counter")
    model.Delta_0(0)
    return model


@pytest.fixture
def coupled_model():
    root = CoupledDEVS("root")
    inner = CoupledDEVS("inner")

    model_a = CounterModel("A", delay=5)
    model_b = CounterModel("B", delay=10)

    inner.add_child(model_a)
    inner.add_child(model_b)
    root.add_child(inner)

    root.Delta_0(0)

    return root, inner, model_a, model_b


# --------------------------------------------------
# Atomic Model Tests
# --------------------------------------------------

def test_capture_atomic_model(atomic_model):
    model = atomic_model
    model.value = 10
    model.data["items"].append(4)
    model.set_timeLast(2)
    model.set_timeNext(12)

    snapshot = model.capture_state()

    assert isinstance(snapshot, ModelSnapshot)
    assert snapshot.model_id == id(model)
    assert snapshot.model_path == model.path

    assert snapshot.model_state["value"] == 10
    assert snapshot.model_state["data"]["items"] == [1, 2, 3, 4]

    assert snapshot.common_state["time_last"] == 2
    assert snapshot.common_state["time_next"] == 12


def test_restore_atomic_model(atomic_model):
    model = atomic_model

    model.value = 10
    model.data["items"].append(4)
    model.set_timeLast(2)
    model.set_timeNext(12)

    snapshot = model.capture_state()

    model.value = 99
    model.data["items"].clear()
    model.set_timeLast(50)
    model.set_timeNext(60)

    model.restore_state(snapshot)

    assert model.value == 10
    assert model.data["items"] == [1, 2, 3, 4]
    assert model.timeLast() == 2
    assert model.timeNext() == 12


# --------------------------------------------------
# Statistics Tests
# --------------------------------------------------

def test_capture_restore_statistics(atomic_model):
    model = atomic_model

    model.t_last_update = 3
    model.StatePercentage = {
        "idle": 3.0,
        "busy": 7.0,
    }

    snapshot = model.capture_state()

    model.t_last_update = 100
    model.StatePercentage = {"other": 50.0}

    model.restore_state(snapshot)

    assert model.t_last_update == 3
    assert model.StatePercentage == {
        "idle": 3.0,
        "busy": 7.0,
    }


# --------------------------------------------------
# Snapshot Independence Tests
# --------------------------------------------------

def test_snapshot_independence(atomic_model):
    model = atomic_model

    model.value = 10
    model.data = {"items": [1, 2, 3]}
    model.StatePercentage = {"idle": 5.0}

    snapshot = model.capture_state()

    model.value = 20
    model.data["items"].append(4)
    model.StatePercentage["idle"] = 100.0

    assert snapshot.model_state["value"] == 10
    assert snapshot.model_state["data"]["items"] == [1, 2, 3]
    assert snapshot.common_state["state_percentage"] == {
        "idle": 5.0,
    }


# --------------------------------------------------
# Coupled Model Tests
# --------------------------------------------------

def test_capture_coupled_model(coupled_model):
    root, inner, model_a, model_b = coupled_model

    model_a.value = 10
    model_b.value = 20

    snapshot = root.capture_state()

    assert snapshot.model_id == id(root)

    assert "inner" in snapshot.children

    inner_snapshot = snapshot.children["inner"]

    assert inner_snapshot.model_id == id(inner)
    assert inner_snapshot.children["A"].model_state["value"] == 10
    assert inner_snapshot.children["B"].model_state["value"] == 20


def test_restore_coupled_model(coupled_model):
    root, inner, model_a, model_b = coupled_model

    model_a.value = 10
    model_b.value = 20

    snapshot = root.capture_state()

    model_a.value = 100
    model_b.value = 200

    model_a.set_timeNext(30)
    model_b.set_timeNext(40)

    inner.update_tN()
    root.update_tN()

    root.restore_state(snapshot)

    assert model_a.value == 10
    assert model_b.value == 20

    assert model_a.timeNext() == 5
    assert model_b.timeNext() == 10

    assert root.timeNext() == 5
    assert inner.timeNext() == 5


def test_restore_rebuilds_imm(coupled_model):
    root, inner, model_a, model_b = coupled_model

    snapshot = root.capture_state()

    assert inner.IMM == {model_a}

    model_a.set_timeNext(20)
    model_b.set_timeNext(5)

    inner.update_tN()
    root.update_tN()

    assert inner.IMM == {model_b}

    root.restore_state(snapshot)

    assert inner.IMM == {model_a}


# --------------------------------------------------
# Snapshot Validation Tests
# --------------------------------------------------

def test_restore_rejects_invalid_snapshot_type(atomic_model):
    with pytest.raises(TypeError):
        atomic_model.restore_state({})


def test_restore_rejects_model_mismatch(atomic_model):
    another_model = CounterModel("another")
    another_model.Delta_0(0)

    snapshot = atomic_model.capture_state()

    with pytest.raises(ValueError, match="Snapshot model mismatch"):
        another_model.restore_state(snapshot)


def test_restore_rejects_hierarchy_mismatch(coupled_model):
    root, inner, model_a, model_b = coupled_model

    snapshot = root.capture_state()

    extra = CounterModel("extra")
    root.add_child(extra)

    model_a.value = 99

    with pytest.raises(ValueError, match="snapshot hierarchy"):
        root.restore_state(snapshot)

    # Validation must fail before changing the model state.
    assert model_a.value == 99