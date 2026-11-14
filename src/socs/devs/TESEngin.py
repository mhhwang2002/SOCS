#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

"""
Simulation engine for TES models.
Originally written in 2011 and modernized for Python 3.
"""

import sys
import threading
import time
from copy import deepcopy

from .TESBase import PortValue, TES
from .Base import exception_handling
from .AlarmClock import AlarmClock
from .Snapshot import SystemSnapshot
from enum import Enum, auto
from collections import deque
import math

class ExecutionType(Enum):
    CONTINUOUS = 1 # when continuous model updates
    ANIMATION = 2  # when Animation updates 
    X_BUFFER = 3   # when external inputs exist in buffer
    DEVS = 4       # when DEVS event happens 
    END = 5        # when time reaches simulation end time 

class ExecutionResult(Enum):
    NoEventOccurred = 1           # when continuous/animation execution occurs but no event happens
    UnscheduledEventOccurred  =2  # when continuous/animation execution occurs but an unscheduled event happens
    ExternalEventOccurred = 3     # when an external event occurs
    ScheduledEventOccurred = 4    # when DEVS event happens
    TimeBoundReached = 5          # when time reaches simulation end time 

class TESEnginBase:
    """Synchronous simulation engine for a TES model."""
    def __init__(self, sim, inject_CBF=None, 
                    TimeIntervalContinuous=float("inf"), 
                    TimeIntervalAnimation=float("inf"), TimeSimulationEnd=float("inf")):
        super().__init__()
        if not isinstance(sim, TES):
            raise TypeError(f"{sim!s} is not a TES")

        self.model = sim
        self.inject_CBF = inject_CBF

        self.SkipLoop = True
        self.ToBeKilled = False
        self.t_segment_start = self.t_segment_end = time.perf_counter()
        self.tNow = 0.0
        self.XBuffer = deque()
        self.sfactor = 1.0
        self.StepByStep = False
        self.TimeIntervalContinuous = TimeIntervalContinuous
        self.TimeIntervalAnimation = TimeIntervalAnimation
        self._tSimBound = TimeSimulationEnd 
        self._tNextContinuous = float("inf")
        self._tNextAnimation = float("inf")
        self.bVerbose = True
        self.iSimRun = 1
        self.bAutoresetByTimebound = False
        self.TotalSimRuns = 1
        self.TrajStatistics = []
        self.last_progress = 0.0
        self.reset("")

    def reset(self, command=""):
        """Reset the simulation model and engine state."""
        self.last_progress = 0.0
        self.tNow = 0.0
        if self.model is not None:
            self.model.set_init_total_state(self.tNow)
            self._tNextContinuous = self.tNow + self.TimeIntervalContinuous

        self.execute_animation(self.tNow, self.tNow)
        self.XBuffer.clear()

        if len(command) > 1 and command[1] == "-a":
            self.TrajStatistics = []
            self.iSimRun = 1

    def _ensure_snapshot_safe(self):
        """Require the engine to be paused before snapshot operations."""
        if not self.SkipLoop:
            raise RuntimeError(
                "Snapshot capture and restore require a paused simulation engine."
            )

    def _collect_models_by_identity(self) -> dict[int, TES]:
        """Return all models in the current hierarchy keyed by Python identity."""
        models = {}

        def visit(model):
            model_identity = id(model)
            if model_identity in models:
                raise ValueError("The model hierarchy contains a repeated object.")
            models[model_identity] = model

            for child in model.children:
                if not isinstance(child, TES):
                    raise TypeError(
                        f"{model.path}: child {child.name!r} is not TES."
                    )
                visit(child)

        visit(self.model)
        return models

    def capture_snapshot(self) -> SystemSnapshot:
        """Capture model state, engine schedules, and pending external inputs."""
        self._ensure_snapshot_safe()
        if self.model is None:
            raise RuntimeError("Cannot capture a snapshot without a root model.")

        models_by_identity = self._collect_models_by_identity()
        x_buffer = []

        for target_model, input_value in self.XBuffer:
            if target_model is None:
                target_model_id = None
            else:
                target_model_id = id(target_model)
                if target_model_id not in models_by_identity:
                    raise ValueError(
                        "XBuffer contains a target model outside the root hierarchy."
                    )
            x_buffer.append((target_model_id, deepcopy(input_value)))

        return SystemSnapshot(
            t_now=self.tNow,
            root_model=self.model.capture_state(),
            t_next_continuous=self._tNextContinuous,
            t_next_animation=self._tNextAnimation,
            x_buffer=x_buffer,
        )

    def restore_snapshot(self, snapshot: SystemSnapshot) -> None:
        """Restore model state, engine schedules, and pending external inputs."""
        self._ensure_snapshot_safe()
        if not isinstance(snapshot, SystemSnapshot):
            raise TypeError("snapshot must be a SystemSnapshot instance.")
        if self.model is None:
            raise RuntimeError("Cannot restore a snapshot without a root model.")

        # Resolve and copy queued inputs before changing the current model state.
        models_by_identity = self._collect_models_by_identity()
        restored_x_buffer = deque()

        for target_model_id, input_value in snapshot.x_buffer:
            if target_model_id is None:
                target_model = None
            else:
                try:
                    target_model = models_by_identity[target_model_id]
                except KeyError as exc:
                    raise ValueError(
                        "Snapshot XBuffer refers to a model outside the current hierarchy."
                    ) from exc

            restored_x_buffer.append((target_model, deepcopy(input_value)))

        # Validate the full model hierarchy before changing any model state.
        self.model._validate_snapshot(snapshot.root_model)

        # Restore model state first; model restoration may recompute schedules.
        self.model.restore_state(snapshot.root_model)

        # Restore engine values last so recomputation cannot overwrite them.
        self.tNow = snapshot.t_now
        self._tNextContinuous = snapshot.t_next_continuous
        self._tNextAnimation = snapshot.t_next_animation
        self.XBuffer = restored_x_buffer
 

    def set_simulation_time_bound(self, end_time: float) -> None:
        """Set the absolute simulation time bound."""
        self._ensure_snapshot_safe()

        if isinstance(end_time, bool) or not isinstance(end_time, (int, float)):
            raise TypeError("end_time must be a real number.")

        if not math.isfinite(end_time):
            raise ValueError("end_time must be finite.")

        if end_time < self.tNow:
            raise ValueError(
                "end_time cannot be earlier than the current simulation time."
            )

        self._tSimBound = float(end_time)

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

    def do_animation(self):  # Override this method to implement animation logic
        pass

    def execute_animation(self, tNow, prevStepExeTime):
        self.do_animation()
        self._tNextAnimation = tNow + self.TimeIntervalAnimation
        return ExecutionResult.NoEventOccurred

    def execute_continuous(self, tNow):
        self._tNextContinuous = tNow + self.TimeIntervalContinuous
        return ExecutionResult.NoEventOccurred

    def execute_devs_star(self,tNow):
        if(self.model is not None):
            src_str = self.model.qstate_to_s() if self.bVerbose else ""
            z = self.model.Delta_y(tNow)  # execute DEVS* transition  
            self.print_src_and_event_segment(src_str, z, tNow) if self.bVerbose else None
            #self._NbDEVSSExecutions += 1
        return ExecutionResult.ScheduledEventOccurred        

    def execute_devs_x(self, tNow, target_md, ipv):
        self.print_src_and_event_segment(
                    self.model.qstate_to_s(), ipv, tNow) if self.bVerbose else None
        target_md.Delta_x(ipv, self.tNow) if target_md is not None else None
        if target_md is not self.model and target_md.parent is not None:
            target_md.parent.update_tn(update_parent=True)
            self._tLastDEVS = self.tNow  # update last one
            self._tNextDEVS = self.model.tNext  # update next one
            #self._NbDEVSSExecutions += 1
        return ExecutionResult.ExternalEventOccurred
    
    def pick_next_execution_time_type(self):
        if(len(self.XBuffer) > 0):
            return self.tNow, ExecutionType.X_BUFFER
        tNextDEVS = self.model.tNext    
        next_time = min(self._tNextContinuous, self._tNextAnimation, tNextDEVS, self._tSimBound)
        if(next_time == self._tSimBound):
            return next_time, ExecutionType.END
        if(next_time == self._tNextContinuous):
            return next_time, ExecutionType.CONTINUOUS
        if(next_time == self._tNextAnimation):
            return next_time, ExecutionType.ANIMATION
        if(next_time == tNextDEVS):
            return next_time, ExecutionType.DEVS 

    def execute_one_step(self):
        """Execute one simulation step."""
        prevStepExeTime = self.tNow
        executionResult = ExecutionResult.NoEventOccurred
        next_time, next_type = self.pick_next_execution_time_type()

        unscheduled = False
        if math.isfinite(next_time) and next_time > self.tNow:
            unscheduled = self.model.Delta_x(None, next_time)
            self.tNow = next_time
        if next_type != ExecutionType.END:
            next_time, next_type = self.pick_next_execution_time_type()
            if next_time != self.tNow:
                return (
                    ExecutionResult.UnscheduledEventOccurred
                    if unscheduled else ExecutionResult.NoEventOccurred
                )

        if next_type == ExecutionType.X_BUFFER:
            target_md, ipv = self.XBuffer.popleft()  
            executionResult = self.execute_devs_x(self.tNow, target_md, ipv)
        elif next_type == ExecutionType.DEVS:
            executionResult =self.execute_devs_star(self.tNow)
        elif next_type == ExecutionType.CONTINUOUS:
            executionResult = self.execute_continuous(self.tNow)
        elif next_type == ExecutionType.ANIMATION:
            executionResult = self.execute_animation(self.tNow, prevStepExeTime)
        elif next_type == ExecutionType.END:
            self.model.update_statistics(self.tNow)
            print(
                "\nNext schedule reaches the simulation time bound " 
                f"{next_time:.3f}."
            ) 
            executionResult = ExecutionResult.TimeBoundReached
            
        if unscheduled and executionResult == ExecutionResult.NoEventOccurred:
            return ExecutionResult.UnscheduledEventOccurred
        return executionResult

    def execute_steps(self):
        """Execute simulation steps until a pause is needed."""
        executionResult = ExecutionResult.NoEventOccurred
        while executionResult is not ExecutionResult.TimeBoundReached:
            executionResult = self.execute_one_step()
            self.display_q()

def find_cmd_index(command, text):
    """Return the index of text in command, or -1 if it is absent."""
    for index, item in enumerate(command):
        if item == text:
            return index
    return -1

class TESEnginThread(threading.Thread, TESEnginBase):
    """Run a TESEnginBase instance in a dedicated thread."""
    def __init__(self, sim, inject_CBF=None, 
                 TimeIntervalContinuous=float("inf"), 
                 TimeIntervalAnimation=float("inf"), TimeSimulationEnd=float("inf")):
        super().__init__()
        TESEnginBase.__init__(self, sim, inject_CBF, TimeIntervalContinuous, TimeIntervalAnimation, TimeSimulationEnd)  

    def exp_handling(self):
        """Report the active exception and pause the simulation."""
        print(
            f"Exception {sys.exc_info()[0]} occurs at time {self.tNow:.3f}."
        )
        exception_handling()
        self.pause()

    
    def run(self):
        """Run the simulation loop in this thread."""
        while not self.ToBeKilled:
            if not self.SkipLoop:
                try:
                    executionResult = self.execute_one_step()
                    if executionResult is not ExecutionResult.NoEventOccurred and self.StepByStep:
                        self.pause()
                    self.display_q()
                    if executionResult is ExecutionResult.TimeBoundReached:
                        if self.bAutoresetByTimebound:
                            self.TrajStatistics.append(
                                self.model.get_statistics_str()
                            )
                            self.reset("")
                            print(f"Iter={self.iSimRun}")
                            if self.iSimRun < self.TotalSimRuns:
                                self.iSimRun += 1
                                self.make_run()
                            else:
                                print("############################################")
                                print("      Simulation Run is completed!")
                                print("############################################")    
                except Exception:
                    self.exp_handling()
                time.sleep(0.001)
            else:
                time.sleep(0.01)

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
        print(f"display time mode = {AlarmClock.DisplayMode}, -dtm")
        print(f"Verbose mode = {self.bVerbose}, -vb")
        print(f"simulation time bound = {self._tSimBound:.3f}, -stb time")
        print(f"Autoreset mode by timebound = {self.bAutoresetByTimebound}, -ar")
        print(f"Total#ofSimRuns = {self.TotalSimRuns}, -tsr n")
        print()

    def change_mode(self, command):
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

        i = find_cmd_index(command, "-stb")
        if i > -1 and len(command) > i + 1:
            stb = command[i + 1]
            self._tSimBound = float("inf") if stb == "inf" else float(stb)
            print(f"new stb={self._tSimBound:.3f}")

        i = find_cmd_index(command, "-ar")
        if i > -1:
            self.bAutoresetByTimebound = not self.bAutoresetByTimebound
            print(
                "The new Autoreset by timebound = "
                f"{self.bAutoresetByTimebound}."
            )

        i = find_cmd_index(command, "-tsr")
        if i > -1 and len(command) > i + 1:
            number = int(command[i + 1])
            if number > 0:
                self.TotalSimRuns = number
                print(f"Total#ofSimRuns = {self.TotalSimRuns}.")

        i = find_cmd_index(command, "-speed")
        if i > -1 and len(command) > i + 1:
            scale_factor = float(command[i + 1])
            if scale_factor > 0:
                self.sfactor = scale_factor
                print(f"The new scale factor = {self.sfactor:.3f}.")

        if find_cmd_index(command, "-step") > -1:
            self.toggle_step_by_step()
        if find_cmd_index(command, "-dtm") > -1:
            self.set_time_display_mode()
        if find_cmd_index(command, "-vb") > -1:
            self.bVerbose = not self.bVerbose
            print(f"New Verbose = {self.bVerbose}")

    def pause(self):
        """Pause the simulation loop at the current simulation time."""
        if not self.SkipLoop:
            self.t_segment_end = self.tNow
            self.SkipLoop = True

    def inject(self, target_model=None, input_value=None):
        """Queue an external input, or use the interactive injection callback.

        Direct use queues the event without starting the simulation loop:
            engine.inject(target_model, PortValue(target_model.IP["?port"], value))

        Calling inject() without arguments preserves the interactive menu behavior.
        """
        if target_model is not None or input_value is not None:
            if target_model is None or input_value is None:
                raise ValueError("target_model and input_value must be provided together.")
            if not isinstance(target_model, TES):
                raise TypeError("target_model must be a TES instance.")
            if not isinstance(input_value, PortValue):
                raise TypeError("input_value must be a PortValue instance.")
            if input_value.port not in target_model.IP:
                raise ValueError("The input port must be registered on target_model.")
            if id(target_model) not in self._collect_models_by_identity():
                raise ValueError("target_model is outside the simulation hierarchy.")

            self.XBuffer.append((target_model, input_value))
            return

        if self.inject_CBF is None:
            print("no callback function defined\n")
            return

        try:
            target_md, pvalue = self.inject_CBF(self.model)
            if pvalue is not None:
                self.inject(target_md, pvalue)
                self.make_run()
            else:
                print("None event generated!")
        except Exception:
            self.exp_handling()

    def toggle_step_by_step(self):
        """Toggle step-by-step mode."""
        self.StepByStep = not self.StepByStep
        print(f"New StepByStep mode = {self.StepByStep}")

    def terminate(self):
        """Stop the simulation thread and wait for it to finish."""
        self.ToBeKilled = True
        if self.is_alive() and threading.current_thread() is not self:
            self.join()

    def set_speed_factor(self):
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

    def set_time_display_mode(self):
        """Toggle the simulation time display mode."""
        try:
            AlarmClock.toggle_display_mode()
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

    def display_progress(self):
        """Display a simple progress indicator."""
        if self._tSimBound in (0, float("inf")):
            return
        progress = (self.tNow / self._tSimBound) * 100.0
        diff_progress = int(progress / 10) - int(self.last_progress / 10)
        self.last_progress = progress
        if diff_progress > 0:
            sys.stdout.write(".")
            sys.stdout.flush()

    def display_q(self):
        """Display the current total state and simulation time."""
        if self.bVerbose:
            print("-- the current state-- ")
            print(f"{self.model.qstate_to_s()}\t at time {self.tNow:.3f}.")
        else:
            self.display_progress()

    def display_help(self):
        """Display the simulation engine version/help."""
        return None

    def run_interactive_menu(self):
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
                    self.change_mode(command)
                elif action in ("exit", "x"):
                    self.terminate()
                    break
                else:
                    print(f"Invalid input = {action}")
            except Exception:
                self.exp_handling()

        self.terminate()


if __name__ == "__main__":
    print("TESEngin module syntax check passed.")
