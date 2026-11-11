"""Precompute DEVS movement and fork-cycle timing for the two-fork crane."""

from __future__ import annotations

import argparse

from .timing import DEFAULT_TIMING_PROFILE, save_timing_profile


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        default=str(DEFAULT_TIMING_PROFILE),
        help="JSON file for the simulated timing profile",
    )
    args = parser.parse_args(argv)
    path, profile = save_timing_profile(args.output)
    print(f"Saved DEVS timing profile: {path}")
    print(f"Station-to-cell and cell-to-station legs: {len(profile['poses']) * 4}")
    print(f"Fork Lift-to-Fork Down cycles: {len(profile['poses']) * 2}")
    print(f"DEVS timing events simulated: {profile['simulation_events']}")
    print(f"Fork approach time: {profile['fork_approach_time']:.3f}")


if __name__ == "__main__":
    main()
