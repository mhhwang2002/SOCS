from uuid import UUID

from socs.devs.TESBase import TES


def test_model_id_initially_none():
    model = TES("Model")

    assert model.model_id is None
    assert model._model_id is None


def test_model_id_property_returns_assigned_uuid():
    model = TES("Model")
    expected_id = UUID("12345678-1234-5678-1234-567812345678")

    model._model_id = expected_id

    assert model.model_id == expected_id