#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

class AlarmClock:
   """Manage the model clock and its display mode."""

   DisplayMode = "tL and tN"

   def __init__(self, tl=0, tN=float('inf')):
      self.tLast = tl
      self.tNext = tN
      self.tNow = tl

   def time_remaining(self, tNow=None):
      """Return time until the next event from now or a supplied time."""
      time = self.tNow if tNow is None else tNow
      return self.tNext - time

   def time_elapsed(self, tNow=None):
      """Return time since the scheduling reference at now or a supplied time."""
      time = self.tNow if tNow is None else tNow
      return time - self.tLast

   def validate_event_time(self, t, *, internal=False):
      """Validate the time of an external input or scheduled internal event."""
      last = self.tLast
      next_time = self.tNext
      if internal:
         if t != next_time:
            raise ValueError(f"Delta_y scheduling error: t={t}, tN={next_time}.")
      elif not last <= t <= next_time:
         raise ValueError(f"Delta_x scheduling error: tL={last}, t={t}, tN={next_time}")

   def format_clock(self):
      """Format the clock for model and engine displays."""
      if AlarmClock.DisplayMode == "tR":
         return f"tR={self.time_remaining():.3f}"
      return f"tL={self.tLast:.3f},tN={self.tNext:.3f}"

   def __str__(self):
      return self.format_clock()

   @staticmethod
   def toggle_display_mode():
      if AlarmClock.DisplayMode == "tL and tN":
         AlarmClock.DisplayMode = "tR"
      else:
         AlarmClock.DisplayMode = "tL and tN"


if __name__ == "__main__":
   manager = AlarmClock(10, 20)
   print(manager)
   print(
      "tNow=15, "
      f"remaining={manager.time_remaining(15)}, elapsed={manager.time_elapsed(15)}"
   )
