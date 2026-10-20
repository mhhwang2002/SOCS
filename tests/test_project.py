import socs


def test_package_import():
    assert socs.__version__ == "0.1.0"


def test_project_identity():
    assert socs.__doc__ is not None
    assert "Simulation and Optimization" in socs.__doc__
