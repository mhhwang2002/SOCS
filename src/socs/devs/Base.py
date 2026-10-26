#######################################################
# A set of useful functions
# Author: moon.hwang@gmail.com
#   Date: Oct.26.2026
#######################################################

import inspect
import sys, traceback
#from odict import odict
#from hashabledict import hashabledict
#from XSY.hashdict import *

def isnumeric(v):
   '''check if v is integer or float'''
   return isinstance(v, int) or isinstance(v, float) #type(1)==type(v) or type(1.1)==type(v)

def function_name():
   '''get function name'''
   caller = inspect.stack()[1]
   return caller[3]

def __line__():
   '''get line number'''
   caller = inspect.stack()[1]
   return int (caller[2])

def line():
   '''get line number'''
   caller = inspect.stack()[1]
   return int (caller[2])

def classname(self):
   '''get class name'''
   return str(self.__class__)#.split("__main__.")[1]

def exception_handling():
   ''' exception handling function '''
   exc_type, exc_value, exc_traceback = sys.exc_info()
   print ("*** print_tb:")
   traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
   print ("*** print_exception:")
   traceback.print_exception(exc_type, exc_value, exc_traceback,
                                        limit=2, file=sys.stdout)
   print ("*** print_exc:")
   traceback.print_exc()
   print ("*** format_exc, first and last line:")
   formatted_lines = traceback.format_exc().splitlines()
   print (formatted_lines[0])
   print (formatted_lines[-1])
   print ("*** format_exception:")
   print (repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
   print ("*** extract_tb:")
   print (repr(traceback.extract_tb(exc_traceback)))
   print ("*** format_tb:")
   print (repr(traceback.format_tb(exc_traceback)))
   print ("*** tb_lineno:", exc_traceback.tb_lineno)


def dict_to_s(obj, attr={}):
   '''convert dictionary key and value into a string'''
   if isinstance(obj, dict):
      labelstr = ""
      first = True
      for k in obj.keys():
         if not first: labelstr += ", "
         first = False;
         #if not( S_hide_keys in attr and k in attr[S_hide_keys]):
         labelstr += str(k) +": "

         if isinstance(obj[k], dict): labelstr += "{"
         labelstr += dict_to_s(obj[k])
         if isinstance(obj[k], dict): labelstr += "}"
      return labelstr
   else:
      return str(obj)

from copy import deepcopy

def cross_product_dict(S, s0):
   '''S is the dictionary'''
   SVL = []
   SVL.append(s0)
   for i in S.keys(): # for each set
      SVLTmp=[]
      for sv in SVL: # for each state vector list
         tmplist = []
         for s in S[i]: # for each element of each set
            newsv = dict(sv) # odict(sv) #deepcopy(sv) # copy the state vector
            newsv[i] = deepcopy(s) # add the element of the set
            tmplist.append(newsv)

         #print 'tmplist=',tmplist
         for elem in tmplist: # copy all in tmplist in SVLTmp
            SVLTmp.append(elem)
      #print 'SVLtmp=',SVLTmp

      SVL = [] # copy all in SVLTmp into SVL
      for elem in SVLTmp:
         SVL.append(elem)

   return SVL

def cross_product_dict_dict(S, s0):
   '''S is the dictionary of dictionary'''
   SVL = []
   SVL.append(s0)
   for i in S.keys(): # for each set
      SVLTmp=[]
      for sv in SVL: # for each state vector list
         tmplist = []
         for k in S[i].keys(): # for each key of S[i]
            newsv = dict(sv) # odict(sv) #deepcopy(sv) # copy the state vector
            newsv[i] = (k,deepcopy(S[i][k])) # add the pair of (key, and value)
            tmplist.append(newsv)

         #print 'tmplist=',tmplist
         for elem in tmplist: # copy all in tmplist in SVLTmp
            SVLTmp.append(elem)
      #print 'SVLtmp=',SVLTmp

      SVL = [] # copy all in SVLTmp into SVL
      for elem in SVLTmp:
         SVL.append(elem)

   return SVL

if __name__ == '__main__':
   S = {1:['A','B','C'], 2:['D', 'E'], 3:['F', 'G', 'H'], 4:['i','j']}
   CP = cross_product_dict(S, dict())

   print ('S=', S, '\n')

   print ('cross_dictionary of S =')
   for elem in CP:
      print (elem)
   print ('|cross_product of S|=',len(CP))

