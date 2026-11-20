"""Incremental ten-robot routes with safely merged execution stages."""
from .incremental_routing import run_example

if __name__ == '__main__':
    run_example(10, merge_sweeps=True)
