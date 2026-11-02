
import pytest

from socs.devs.DEVSBase import HierNamedObj


def test_remove_child():
    root = HierNamedObj("Root")
    child = HierNamedObj("Child")

    root.add_child(child)
    root.remove_child(child)

    assert child not in root.children
    assert child.parent is None


def test_remove_child_preserves_subtree():
    root = HierNamedObj("Root")
    child = HierNamedObj("Child")
    grandchild = HierNamedObj("Grandchild")

    root.add_child(child)
    child.add_child(grandchild)

    root.remove_child(child)

    assert child.parent is None
    assert grandchild.parent is child
    assert grandchild in child.children


def test_remove_child_rejects_unrelated_child():
    root = HierNamedObj("Root")
    child = HierNamedObj("Child")

    with pytest.raises(ValueError):
        root.remove_child(child)

    assert child.parent is None
    assert child not in root.children


def test_remove_child_rejects_wrong_parent():
    root_a = HierNamedObj("RootA")
    root_b = HierNamedObj("RootB")
    child = HierNamedObj("Child")

    root_a.add_child(child)

    with pytest.raises(ValueError):
        root_b.remove_child(child)

    assert child.parent is root_a
    assert child in root_a.children


def test_remove_child_rejects_invalid_type():
    root = HierNamedObj("Root")

    with pytest.raises(TypeError):
        root.remove_child("Child")
