"""Atomic DEVS with continuous evolution between discrete transitions."""

from .DEVS import DEVS


class cDEVS(DEVS):
    def delta_cont(self, elapsed):
        """Evaluate continuous state elapsed seconds after timeLast()."""
        raise NotImplementedError("Implement delta_cont(elapsed) in the model.")

    def Delta_t(self, t):
        elapsed = self.time_elapsed(t)
        if elapsed < 0:
            raise ValueError("Continuous state cannot advance backward in time.")
        super().Delta_t(t)
        if elapsed > 0:
            self.delta_cont(elapsed)
        return False
