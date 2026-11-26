from os import environ

import pytest

from examples.RMFS.path_geometry import AABB, Sweep
from examples.RMFS.sweep_backend import SweepBackend


@pytest.mark.skipif(not environ.get("SOCS_RMFS_CUDA_LIBRARY"),
                    reason="Requires the RMFS CUDA library and a GPU")
def test_cuda_sweep_batches_match_cpu(monkeypatch):
    gpu = SweepBackend()
    monkeypatch.delenv("SOCS_RMFS_CUDA_LIBRARY")
    cpu = SweepBackend()

    def box(x, y):
        return AABB((x, y), (x + 1, y + 1))

    candidate = Sweep(2, tuple(box(0, y * 3) for y in range(129)))
    far = Sweep(2, tuple(box(10, y * 3) for y in range(9)))
    empty = Sweep(2, ())
    touch = Sweep(2, (box(1., 384),))
    small_gap = Sweep(2, (box(1. + 5e-10, 384),))
    cases = [
        (empty, (touch,), 0.0, False),
        (candidate, (empty,), 0.0, False),
        (candidate, (far,) * 64, 0.0, False),
        (candidate, (far,) * 64 + (touch,), 0.0, True),
        (candidate, (small_gap,), 0.0, False),
        (candidate, (small_gap,), 1e-9, True),
    ]
    for first, others, tolerance, expected in cases:
        assert gpu.intersects_any(first, iter(others), tolerance) == expected
        assert cpu.intersects_any(first, iter(others), tolerance) == expected
