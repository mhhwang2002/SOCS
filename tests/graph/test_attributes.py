import pytest

from socs.graph.attributes import AttrDict


def test_key_and_attribute_access_are_equivalent():
    attrs = AttrDict()
    attrs["cost"] = 100

    assert attrs.cost == 100
    assert attrs["cost"] == 100


def test_attribute_assignment_updates_dictionary():
    attrs = AttrDict()

    attrs.type = "precedence"

    assert attrs["type"] == "precedence"


def test_key_assignment_is_visible_as_attribute():
    attrs = AttrDict({"cost": 100})

    attrs["cost"] = 200

    assert attrs.cost == 200


def test_initial_values_are_supported():
    attrs = AttrDict({"cost": 100, "time": 10})

    assert attrs.cost == 100
    assert attrs.time == 10


def test_standard_dictionary_methods_work():
    attrs = AttrDict({"cost": 100, "time": 10})

    assert attrs.get("cost") == 100
    assert set(attrs.keys()) == {"cost", "time"}
    assert dict(attrs.items()) == {"cost": 100, "time": 10}


def test_missing_attribute_raises_attribute_error():
    attrs = AttrDict()

    with pytest.raises(AttributeError):
        _ = attrs.unknown


def test_method_name_collision_uses_bracket_notation():
    attrs = AttrDict()
    attrs["items"] = "custom value"

    assert attrs["items"] == "custom value"
    assert callable(attrs.items)


def test_attribute_deletion_removes_key():
    attrs = AttrDict(cost=100)

    del attrs.cost

    assert "cost" not in attrs
    with pytest.raises(AttributeError):
        _ = attrs.cost


def test_missing_attribute_deletion_raises_attribute_error():
    attrs = AttrDict()

    with pytest.raises(AttributeError):
        del attrs.unknown
