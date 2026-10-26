#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

class Clock:
   '''A class for clocks that has the last event time(tL) and the next event time(tN)'''
   tNow = 0 # static variable for the current time.
   DisplayMode = 'tL and tN' # other one can be 'sigma_e'

   def __init__(self, tl=0.0, tn=float('inf')):
      self.tL, self.tN = tl, tn

   def ta(self):
      '''time advance from last schedule time to next event time ta()=tN-tL'''
      return self.tN - self.tL

   def tr(self):
      '''time remaining from the Clock.tNow to the next event time tN'''
      return self.tN - Clock.tNow

   def te(self):
      '''time elapsed from the last update time tL to the Clock.tNow'''
      return Clock.tNow - self.tL

   def __str__(self):
      '''convert itself to a string'''
      s=""
      if Clock.DisplayMode == 'tL and tN':
         if self.tL == float('-inf') or self.tL == float('inf'): s= "tL=%s"  % self.tL
         else: s= "tL=%.3f"% self.tL
         if self.tN == float('inf'): s += ",tN=%s"  % self.tN
         else:                       s += ",tN=%.3f"% self.tN
         return s
      else:
         if self.tr() == float('inf'): s += "tR=%s"  % self.tr()
         else:                         s += "tR=%.3f"% self.tr()
         return s
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

   def timeRemaining(self) :
      '''time remaining to next schedule time from Clock.tNow: tN - Clock.tNow '''
      return  self.clock.tr()

   def timeElapsed(self) :
      '''time elapsed from last update to the time t: t - tL'''
      return self.clock.te()

   def timeNext(self):
      '''time for next schedule: tN '''
      return self.clock.tN

   def timeLast(self):
      '''time for last update: tL'''
      return self.clock.tL

   def timeAdvance(self):
      '''time advance to tN from the last event time tL: tN - tL'''
      return self.clock.ta()

   def __str__(self):
      return str(self.clock)
# of class TimeManagement

if __name__ == '__main__' :
   print("-------- Clock and Manager test -------- ")
   c = Clock(10)
   print(c)
   Clock.tNow = 15
   print("tNow=%s, ta=%s, tr()=%s, te()=%s" % (Clock.tNow, c.ta(), c.tr(), c.te()))
   print()


   tm = Manager(10, 20)
   print(tm)
   print("tm.ta()=%s, tm.tr()=%s, tm.te()=%s" % (tm.timeAdvance(), tm.timeRemaining(), tm.timeElapsed()))
