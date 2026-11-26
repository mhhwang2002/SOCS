"""Batch RMFS sweep intersections using an optional native CUDA library."""

import ctypes
from itertools import islice
from os import environ
from pathlib import Path


class SweepBackend:
    def __init__(self):
        path = environ.get("SOCS_RMFS_CUDA_LIBRARY")
        self.library = None
        if path:
            try:
                self.library = ctypes.CDLL(str(Path(path).resolve()))
                self.library.rmfs_cuda_error.argtypes = [ctypes.c_int]
                self.library.rmfs_cuda_error.restype = ctypes.c_char_p
                self.library.rmfs_cuda_check_device.argtypes = []
                self.library.rmfs_cuda_check_device.restype = ctypes.c_int
                self.library.rmfs_cuda_intersects.argtypes = [
                    ctypes.POINTER(ctypes.c_double), ctypes.c_size_t, ctypes.c_size_t,
                    ctypes.c_double, ctypes.POINTER(ctypes.c_int),
                ]
                self.library.rmfs_cuda_intersects.restype = ctypes.c_int
            except (OSError, AttributeError) as error:
                raise RuntimeError(f"Cannot load RMFS CUDA library {path!r}: {error}") from error
            self._check(self.library.rmfs_cuda_check_device())

    def _check(self, status):
        if status:
            message = self.library.rmfs_cuda_error(status).decode()
            raise RuntimeError(f"RMFS CUDA error ({status}): {message}")

    def intersects_any(self, candidate, occupied, tolerance):
        """Check the candidate against an iterable of 2D rectangular sweeps."""
        if self.library is None:
            return any(candidate.intersects(sweep, tolerance) for sweep in occupied)
        first = candidate.regions
        if not first:
            return False
        occupied = iter(occupied)
        while batch := tuple(islice(occupied, 64)):
            second = tuple(box for sweep in batch for box in sweep.regions)
            if not second:
                continue
            packed = (ctypes.c_double * (4 * (len(first) + len(second))))(
                *(value for box in (*first, *second) for value in (*box.lower, *box.upper))
            )
            hit = ctypes.c_int()
            self._check(self.library.rmfs_cuda_intersects(
                packed, len(first), len(second), tolerance, ctypes.byref(hit)
            ))
            if hit.value:
                return True
        return False
