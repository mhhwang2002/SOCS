
from uuid import UUID, uuid4

import pytest

from socs.devs.ModelRegistry import ModelRegistry


class SampleModel:
    def __init__(self, name):
        self.name = name
        self.children = []
        self._model_id = None

    def add_child(self, child):
        self.children.append(child)


def test_register_single_model():
    registry = ModelRegistry()
    model = SampleModel("Root")

    registry.register(model)

    assert len(registry) == 1
    assert isinstance(model._model_id, UUID)
    assert registry.get(model._model_id) is model


def test_register_model_hierarchy():
    registry = ModelRegistry()

    root = SampleModel("Root")
    child_a = SampleModel("A")
    child_b = SampleModel("B")
    grandchild = SampleModel("A1")

    root.add_child(child_a)
    root.add_child(child_b)
    child_a.add_child(grandchild)

    registry.register(root)

    assert len(registry) == 4
    assert registry.get(root._model_id) is root
    assert registry.get(child_a._model_id) is child_a
    assert registry.get(child_b._model_id) is child_b
    assert registry.get(grandchild._model_id) is grandchild


def test_register_preserves_existing_uuid():
    registry = ModelRegistry()
    model = SampleModel("Model")
    original_id = uuid4()
    model._model_id = original_id

    registry.register(model)

    assert model._model_id == original_id
    assert registry.get(original_id) is model


def test_register_same_model_twice():
    registry = ModelRegistry()
    model = SampleModel("Model")

    registry.register(model)
    original_id = model._model_id

    registry.register(model)

    assert len(registry) == 1
    assert model._model_id == original_id


def test_duplicate_uuid_for_different_models_is_rejected():
    registry = ModelRegistry()

    model_a = SampleModel("A")
    model_b = SampleModel("B")
    shared_id = uuid4()

    model_a._model_id = shared_id
    model_b._model_id = shared_id

    registry.register(model_a)

    with pytest.raises(ValueError):
        registry.register(model_b)

    assert len(registry) == 1
    assert registry.get(shared_id) is model_a


def test_invalid_uuid_type_is_rejected():
    registry = ModelRegistry()
    model = SampleModel("Model")
    model._model_id = "not-a-uuid"

    with pytest.raises(TypeError):
        registry.register(model)

    assert len(registry) == 0


def test_unregister_model_removes_subtree():
    registry = ModelRegistry()

    root = SampleModel("Root")
    child = SampleModel("Child")
    grandchild = SampleModel("Grandchild")

    root.add_child(child)
    child.add_child(grandchild)

    registry.register(root)

    root_id = root._model_id
    child_id = child._model_id
    grandchild_id = grandchild._model_id

    registry.unregister(child)

    assert len(registry) == 1
    assert root_id in registry
    assert child_id not in registry
    assert grandchild_id not in registry

    # UUIDs remain assigned to the model objects.
    assert child._model_id == child_id
    assert grandchild._model_id == grandchild_id


def test_unregister_by_uuid_removes_only_one_model():
    registry = ModelRegistry()

    root = SampleModel("Root")
    child = SampleModel("Child")
    root.add_child(child)

    registry.register(root)

    root_id = root._model_id
    child_id = child._model_id

    registry.unregister(child_id)

    assert root_id in registry
    assert child_id not in registry
    assert len(registry) == 1


def test_rebuild_replaces_registry_contents():
    registry = ModelRegistry()

    old_root = SampleModel("OldRoot")
    registry.register(old_root)
    old_id = old_root._model_id

    new_root = SampleModel("NewRoot")
    new_child = SampleModel("NewChild")
    new_root.add_child(new_child)

    registry.rebuild(new_root)

    assert len(registry) == 2
    assert old_id not in registry
    assert registry.get(new_root._model_id) is new_root
    assert registry.get(new_child._model_id) is new_child


def test_register_rejects_repeated_model_reference():
    registry = ModelRegistry()

    root = SampleModel("Root")
    child = SampleModel("Child")

    root.add_child(child)
    root.add_child(child)

    with pytest.raises(ValueError):
        registry.register(root)

    assert len(registry) == 0


def test_register_rejects_cycle():
    registry = ModelRegistry()

    root = SampleModel("Root")
    child = SampleModel("Child")

    root.add_child(child)
    child.add_child(root)

    with pytest.raises(ValueError):
        registry.register(root)

    assert len(registry) == 0


def test_get_unregistered_model_raises_key_error():
    registry = ModelRegistry()

    with pytest.raises(KeyError):
        registry.get(uuid4())

def test_duplicate_uuid_inside_hierarchy_is_rejected():
    registry = ModelRegistry()

    root = SampleModel("Root")
    child = SampleModel("Child")

    shared_id = uuid4()
    root._model_id = shared_id
    child._model_id = shared_id
    root.add_child(child)

    with pytest.raises(ValueError):
        registry.register(root)

    assert len(registry) == 0