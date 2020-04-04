 #!/usr/bin/env python

import rospy
import numpy as np
from turtlebot_mpc.unicycle_mpc import UnicycleMPC

class TurtleBotMPC(object):
   def __init__(self, cfg):
      self.cfg = cfg
      
      self.T = cfg['mpc']['T']
      self.N = int(cfg['mpc']['N'])
      
      self.xmin = np.array(cfg['mpc']['xmin']).astype(np.float32)
      self.xmax = np.array(cfg['mpc']['xmax']).astype(np.float32)
      self.umin = np.array(cfg['mpc']['umin']).astype(np.float32)
      self.umin = np.array(cfg['mpc']['umax']).astype(np.float32)

      self.Q = np.array(cfg['mpc']['Q']).astype(np.float32)
      self.QN = np.array(cfg['mpc']['QN']).astype(np.float32)
      self.R = np.array(cfg['mpc']['R']).astype(np.float32)
      
      

