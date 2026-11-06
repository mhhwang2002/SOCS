#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

class Clock:
   '''A class for clocks that has the last event time(tL) and the next event time(tN)'''
   DisplayMode = 'tL and tN' # other one can be 'sigma_e'

   def __init__(self, tl=0.0, tn=float('inf')):
      self.tL, self.tN = tl, tn

   def ta(self):
      '''time advance from last schedule time to next event time ta()=tN-tL'''
      return self.tN - self.tL

   def tr(self, tNow):
      '''time remaining from the given tNow to the next event time tN'''
      return self.tN - tNow

   def te(self, tNow):
      '''time elapsed from the last update time tL to the given tNow'''
      return tNow - self.tL

   def format(self, t_now=None):
      '''Format the clock, using t_now when the remaining-time mode is active.'''
      if Clock.DisplayMode == 'tR' and t_now is not None:
         time_remaining = self.tr(t_now)
         if time_remaining == float('inf'):
            return "tR=%s" % time_remaining
         return "tR=%.3f" % time_remaining

      if self.tL == float('-inf') or self.tL == float('inf'):
         result = "tL=%s" % self.tL
      else:
         result = "tL=%.3f" % self.tL
      if self.tN == float('inf'):
         result += ",tN=%s" % self.tN
      else:
         result += ",tN=%.3f" % self.tN
      return result

   def __str__(self):
      '''Return a representation that does not depend on a current time.'''
      return self.format()
   @staticmethod
   def toggleDisplayMode():
      if Clock.DisplayMode == 'tL and tN':
         Clock.DisplayMode = 'tR'
      else:
         Clock.DisplayMode = 'tL and tN'

class Manager:
   '''A class managing a clock'''
   def __init__(self, tl=0, tN=float('inf')):
      ''' having one clock'''
      self.clock = Clock(tl,tN)

   def set_timeLast(self, tl):
      ''' set the last event time '''
      self.clock.tL = tl

   def set_timeNext(self, tN):
      ''' set the next event time '''
      self.clock.tN = tN

   def timeRemaining(self, tNow) :
      '''time remaining to next schedule time from the given tNow: tN - tNow '''
      return  self.clock.tr(tNow)

   def timeElapsed(self, tNow) :
      '''time elapsed from last update to the time t: t - tL'''
      return self.clock.te(tNow)

   def timeNext(self):
      '''time for next schedule: tN '''
      return self.clock.tN

   def timeLast(self):
      '''time for last update: tL'''
      return self.clock.tL

   def timeAdvance(self):
      '''time advance to tN from the last event time tL: tN - tL'''
      return self.clock.ta()

   def format_clock(self, t_now=None):
      '''Format this model's clock, optionally relative to simulation time.'''
      return self.clock.format(t_now)

   def __str__(self):
      return str(self.clock)
# of class TimeManagement

if __name__ == '__main__' :
   print("-------- Clock and Manager test -------- ")
   c = Clock(10)
   print(c) 
   print("tNow=%s, ta=%s, tr()=%s, te()=%s" % (15, c.ta(), c.tr(15), c.te(15)))
   print()


   tm = Manager(10, 20)
   print(tm)
   print("tNow=%s,tm.ta()=%s, tm.tr()=%s, tm.te()=%s" % (15, tm.timeAdvance(), tm.timeRemaining(15), tm.timeElapsed(15)))
