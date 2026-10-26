#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

"""
Simulation engine for Simulatable models.
Originally written in 2011 and modernized for Python 3.
"""

import sys
import threading
import time

from .Simulatable import Simulatable
from .Base import exception_handling
from .DEVSTime import Clock


def findCmdIndex(command, text):
    """Return the index of text in command, or -1 if it is absent."""
    for index, item in enumerate(command):
        if item == text:
            return index
    return -1


class SimEngine(threading.Thread):
    """Real-time execution engine for a Simulatable model."""

    def __init__(self, sim, inject_CBF=None):
        super().__init__()
        if not isinstance(sim, Simulatable):
            raise TypeError(f"{sim!s} is not a Simulatable")

        self.model = sim
        self.inject_CBF = inject_CBF

        self.SkipLoop = True
        self.ToBeKilled = False
        self.t_segment_start = self.t_segment_end = time.perf_counter()
        Clock.tNow = 0.0
        self.XBuffer = []
        self.sfactor = 1.0
        self.StepByStep = False

        self.simTimeBound = float("inf")
        self.bVerbose = True
        self.iSimRun = 1
        self.bAutoresetByTimebound = False
        self.TotalSimRuns = 1
        self.TrajStatistics = []
        self.last_progress = 0.0

        self.reset("")

    def compute_tNow(self):
        """Compute the current simulation time from wall time and event time."""
        if self.sfactor == float("inf"):
            if (
                self.model.timeNext() == float("inf")
                and self.simTimeBound == float("inf")
            ):
                print(
                    "\nNext schedule reaches infinity in discrete-event-only "
                    f"mode at time {Clock.tNow:.3f}."
                )
                self.pause()
            else:
                Clock.tNow = min(self.model.timeNext(), self.simTimeBound)
        else:
            now_real_time = time.perf_counter()
            elapsed_real_time = now_real_time - self.t_segment_start
            scaled_time = self.t_segment_end + elapsed_real_time * self.sfactor
            Clock.tNow = min(
                self.model.timeNext(), scaled_time, self.simTimeBound
            )

        if Clock.tNow >= self.simTimeBound:
            print(
                f"\nSimulation run time is bounded by "
                f"{self.simTimeBound:.3f}.\n"
            )
            Clock.tNow = self.simTimeBound
            if self.bAutoresetByTimebound:
                self.TrajStatistics.append(self.model.get_statistics_str())
                self.reset("")
                print(f"Iter={self.iSimRun}")
                if self.iSimRun < self.TotalSimRuns:
                    self.iSimRun += 1
                else:
                    print("############################################")
                    print("   Multiple Simulation Run is completed!")
                    print("############################################")
                    self.pause()
            else:
                self.pause()

    def exp_handling(self):
        """Report the active exception and pause the simulation."""
        print(
            f"Exception {sys.exc_info()[0]} occurs at time {Clock.tNow:.3f}."
        )
        exception_handling()
        self.pause()

    def print_src_and_event_segment(self, src_state_str, z, tnow):
        """Print the source state and event segment."""
        print(f"\n{src_state_str}\t at time {tnow:.3f}.")
        if not hasattr(z, "__iter__") or isinstance(z, (str, bytes)):
            zstr = str(z)
        elif len(z) == 1:
            zstr = "".join(str(event) for event in z)
        else:
            zstr = "".join(f"({event})" for event in z)
        print(f"\t -- ({zstr},{tnow:.3f}) -->")

    def executeOneStep(self):
        """Execute one simulation step."""
        bNeedToPause = False
        bDESExecuted = False

        if self.XBuffer:
            target_md, ipv = self.XBuffer.pop()
            if self.bVerbose:
                self.print_src_and_event_segment(
                    self.model.qstate_to_s(), ipv, Clock.tNow
                )
            if target_md is not None:
                target_md.Delta_x(ipv, Clock.tNow)
                bDESExecuted = True
                if target_md is not self.model and target_md.parent is not None:
                    target_md.parent.update_tN(update_parent=True)
        else:
            if self.sfactor == float("inf"):
                if (
                    self.model.timeNext() == float("inf")
                    and self.simTimeBound == float("inf")
                ):
                    print(
                        "\nNext schedule reaches infinity in "
                        "discrete-event-only mode at time "
                        f"{Clock.tNow:.3f}."
                    )
                    bNeedToPause = True
                else:
                    Clock.tNow = min(
                        self.model.timeNext(), self.simTimeBound
                    )
            else:
                now_real_time = time.perf_counter()
                elapsed_real_time = now_real_time - self.t_segment_start
                scaled_time = (
                    self.t_segment_end + elapsed_real_time * self.sfactor
                )
                Clock.tNow = min(
                    self.model.timeNext(), scaled_time, self.simTimeBound
                )

            self.model.Delta_null(Clock.tNow)

            if Clock.tNow == self.model.timeNext():
                if self.bVerbose:
                    src_str = self.model.qstate_to_s()
                    tnow = Clock.tNow
                z = self.model.Delta_y(Clock.tNow)
                bDESExecuted = True
                if self.bVerbose:
                    self.print_src_and_event_segment(src_str, z, tnow)
            elif Clock.tNow >= self.simTimeBound:
                print(
                    f"\nSimulation run time is bounded by "
                    f"{self.simTimeBound:.3f}.\n"
                )
                Clock.tNow = self.simTimeBound
                if self.bAutoresetByTimebound:
                    self.TrajStatistics.append(
                        self.model.get_statistics_str()
                    )
                    self.reset("")
                    print(f"Iter={self.iSimRun}")
                    if self.iSimRun < self.TotalSimRuns:
                        self.iSimRun += 1
                    else:
                        print("############################################")
                        print("   Multiple Simulation Run is completed!")
                        print("############################################")
                        bNeedToPause = True
                else:
                    bNeedToPause = True

        return bNeedToPause, bDESExecuted

    def run(self):
        """Run the simulation loop in this thread."""
        while not self.ToBeKilled:
            if not self.SkipLoop:
                try:
                    bNeedToPause, bDESExecuted = self.executeOneStep()
                    if bNeedToPause or (bDESExecuted and self.StepByStep):
                        self.pause()
                    self.display_q()
                except Exception:
                    self.exp_handling()
                time.sleep(0.001)
            else:
                time.sleep(0.01)

    def reset(self, command):
        """Reset the associated model state."""
        self.last_progress = 0.0
        if self.model is not None:
            self.model.Delta_0(0.0)

        self.XBuffer.clear()
        if len(command) > 1 and command[1] == "-a":
            self.TrajStatistics = []
            self.iSimRun = 1

    def make_run(self, command=""):
        """Start or resume the simulation loop."""
        if self.SkipLoop:
            self.t_segment_start = time.perf_counter()
            self.SkipLoop = False

    def display_modes(self):
        """Display simulation mode settings."""
        print("=== (m)ode settings ===")
        print(f"Current Simulation i = {self.iSimRun}")
        print(f"StepByStep mode = {self.StepByStep}, -step")
        print(f"scale factor for speed = {self.sfactor}, -speed sf")
        print(f"display time mode = {Clock.DisplayMode}, -dtm")
        print(f"Verbose mode = {self.bVerbose}, -vb")
        print(f"simulation time bound = {self.simTimeBound:.3f}, -stb time")
        print(f"Autoreset mode by timebound = {self.bAutoresetByTimebound}, -ar")
        print(f"Total#ofSimRuns = {self.TotalSimRuns}, -tsr n")
        print()

    def changeMode(self, command):
        """Change simulation modes using command-line options."""
        if len(command) == 1:
            self.display_modes()
            print(
                "change mode of simulation: autoreset, multiple_runs, "
                "speed, step, time, verbose"
            )
            command = input(
                "use -step, -speed sf, -dtm, -vb, -ar, -tsr no\n>"
            ).split()

        i = findCmdIndex(command, "-stb")
        if i > -1 and len(command) > i + 1:
            stb = command[i + 1]
            self.simTimeBound = float("inf") if stb == "inf" else float(stb)
            print(f"new stb={self.simTimeBound:.3f}")

        i = findCmdIndex(command, "-ar")
        if i > -1:
            self.bAutoresetByTimebound = not self.bAutoresetByTimebound
            print(
                "The new Autoreset by timebound = "
                f"{self.bAutoresetByTimebound}."
            )

        i = findCmdIndex(command, "-tsr")
        if i > -1 and len(command) > i + 1:
            number = int(command[i + 1])
            if number > 0:
                self.TotalSimRuns = number
                print(f"Total#ofSimRuns = {self.TotalSimRuns}.")

        i = findCmdIndex(command, "-speed")
        if i > -1 and len(command) > i + 1:
            scale_factor = float(command[i + 1])
            if scale_factor > 0:
                self.sfactor = scale_factor
                print(f"The new scale factor = {self.sfactor:.3f}.")

        if findCmdIndex(command, "-step") > -1:
            self.toggleStepByStep()
        if findCmdIndex(command, "-dtm") > -1:
            self.setTimeDisplayMode()
        if findCmdIndex(command, "-vb") > -1:
            self.bVerbose = not self.bVerbose
            print(f"New Verbose = {self.bVerbose}")

    def pause(self):
        """Pause the simulation loop and update statistics."""
        if not self.SkipLoop:
            self.t_segment_end = Clock.tNow
            self.model.Delta_null(Clock.tNow)
            self.SkipLoop = True

    def inject(self):
        """Inject an input event through a port using the callback."""
        if self.inject_CBF is None:
            print("no callback function defined\n")
            return

        try:
            target_md, pvalue = self.inject_CBF(self.model)
            if pvalue is not None:
                self.XBuffer.append((target_md, pvalue))
                self.make_run()
            else:
                print("None event generated!")
        except Exception:
            self.exp_handling()

    def toggleStepByStep(self):
        """Toggle step-by-step mode."""
        self.StepByStep = not self.StepByStep
        print(f"New StepByStep mode = {self.StepByStep}")

    def terminate(self):
        """Stop the simulation thread and wait for it to finish."""
        self.ToBeKilled = True
        if self.is_alive() and threading.current_thread() is not self:
            self.join()

    def setSpeedFactor(self):
        """Set the speed factor to a positive number or infinity."""
        try:
            print(
                f"The current scale factor={self.sfactor}, "
                "put a new scale factor."
            )
            inp = input("input a positive number or inf>")
            if inp in ("inf", "'inf'"):
                self.sfactor = float("inf")
            else:
                factor = float(inp)
                if factor > 0:
                    self.sfactor = factor
                else:
                    print("speed factor must be a positive real number or inf")
                    print(f"the new scale factor={self.sfactor}")
        except Exception:
            self.exp_handling()

    def setTimeDisplayMode(self):
        """Toggle the simulation time display mode."""
        try:
            Clock.toggleDisplayMode()
        except Exception:
            self.exp_handling()

    def display_menu(self):
        """Display the interactive menu."""
        print("-- simulation menu --")
        print("e(x)it, (in)ject, (m)ode, (p)ause, reset, run, stat(i)stics>")

    def print_statistics(self, cmd):
        """Print statistics for the current or all trajectories."""
        if len(cmd) > 1 and cmd[1] == "-a":
            for index, statistics in enumerate(self.TrajStatistics, start=1):
                print(f"Iter = {index}\n {statistics}")
                print(
                    "--------------------------------------------------------------- "
                    f"End of Iter = {index}\n"
                )
        else:
            print("=" * 86 + "\n")
            print("statistics of the current trajectory")
            print(self.model.get_statistics_str())
            print("=" * 86 + "\n")

    def displayProgress(self):
        """Display a simple progress indicator."""
        if self.simTimeBound in (0, float("inf")):
            return
        progress = (Clock.tNow / self.simTimeBound) * 100.0
        diff_progress = int(progress / 10) - int(self.last_progress / 10)
        self.last_progress = progress
        if diff_progress > 0:
            sys.stdout.write(".")
            sys.stdout.flush()

    def display_q(self):
        """Display the current total state and simulation time."""
        if self.bVerbose:
            print("-- the current state-- ")
            print(f"{self.model.qstate_to_s()}\t at time {Clock.tNow:.3f}.")
        else:
            self.displayProgress()

    def display_help(self):
        """Display the simulation engine version/help."""
        return None

    def runInteractiveMenu(self):
        """Run the interactive console menu."""
        self.display_help()
        self.display_modes()
        self.start()
        self.display_q()

        while True:
            self.display_menu()
            command = input().split()
            if not command:
                continue

            try:
                action = command[0]
                if action == "run":
                    self.make_run(command)
                elif action == "reset":
                    self.reset(command)
                elif action in ("statistics", "i"):
                    self.print_statistics(command)
                elif action in ("pause", "p"):
                    self.pause()
                elif action in ("in", "inject"):
                    self.inject()
                elif action in ("m", "mode"):
                    self.changeMode(command)
                elif action in ("exit", "x"):
                    self.terminate()
                    break
                else:
                    print(f"Invalid input = {action}")
            except Exception:
                self.exp_handling()

        self.terminate()


if __name__ == "__main__":
    print("SimEngine module syntax check passed.")
