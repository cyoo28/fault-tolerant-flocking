from threading import Thread
from queue import Empty
import numpy as np
import time

class Node(Thread):
  def __init__(self, uid, goal, fail_rate, gains):
    """ Constructor """
    Thread.__init__(self)
    
    # basic information about network connectivity
    self.uid = uid    # node UID (an integer)
    self.out_nbr = [] # list of outgoing edges (see Edge class)
    self.in_nbr = []  # list of incoming edges (see Edge class)
    
    self.position = [0,0]   # position ([rx, ry])
    self.velocity = [0,0]   # position ([vx, vy])
    self.done = False # termination flag
    
    self.nominaldt = 0.05 # desired time step

    # Defining gains for flock centering (coherence), velocity matching (alignment), and collision avoidance
    self.K_coh = gains[0]
    self.K_ali = gains[1]
    self.K_col = gains[2]
    # Defining new gains for target following and correction
    self.K_tar = gains[3]
    self.K_cor = gains[4]
    
    # Defining failure probability and flags
    self.fail_prob = fail_rate
    self.fail_flag = False
    
    # Defining suspected failed agents and previous velocities
    self.sus_fail = []
    self.vel_old = np.array((-1,-1))
    self.vel_older = ((-2,-2))
    
    # Defining target location and landing flag
    self.goal = goal
    self.land_flag = False

    # Defining max speed and breaking gain
    self.max_speed = 3
    self.K_break = 0.8

    # Defining additional relevant metrics
    self.col_count = 0
    self.completion_time = 0
    self.begin = 0
    self.finish = 0
    
  def __str__(self):
    """ Printing """
    return "Node %d has %d in_nbr and %d out_nbr" % (self.uid, len(self.in_nbr), len(self.out_nbr))

  ################################################
  #
  # Modify the graph
  #
  ################################################

  def addOutgoing(self, e):
    """ Add an edge for outgoing messages """
    self.out_nbr.append(e)
    
  def addIncoming(self, e):
    """ Add an edge for incoming messages """
    self.in_nbr.append(e)
    
  ################################################
  #
  # Set states externally
  #
  ################################################

  def setPosition(self, s):
    """ update the position of the node """
    self.position = s
    
  def getPosition(self):
    """ return the position of the node """
    return np.array(self.position)
    
  def setVelocity(self, v):
    """ update the velocity of the node """
    self.velocity = v
    
  def getVelocity(self):
    """ return the velocity of the node """
    return np.array(self.velocity)

  def terminate(self):
    """ stop sim """
    self.done = True
 
  ################################################
  #
  # Run the vehicle
  #
  ################################################

  def run(self):
    """ Send messages, Retrieve message, Transition """
    self.begin = time.time()
    while (not self.done):
      time1 = time.time()
      self.send()
      self.transition()
      
      self.systemdynamics()
      
      time2 = time.time()
      time.sleep(max(self.nominaldt - (time2-time1), 0))
    
  def systemdynamics(self):
    """ Update the state based on dynamics """
    self.position = (self.position + self.velocity * self.nominaldt)

    # Don't need to wrap the world anymore
    """
    if ( (self.position[0] > 1.0) and (self.velocity[0] > 0.0)) :
      self.position[0] -= 1
    if ( (self.position[0] < 0.0) and (self.velocity[0] < 0.0)) :
      self.position[0] += 1
    if ( (self.position[1] > 1.0) and (self.velocity[1] > 0.0)) :
      self.position[1] -= 1
    if ( (self.position[1] < 0.0) and (self.velocity[1] < 0.0)) :
      self.position[1] += 1
    """
      
  ################################################
  #
  # YOUR CODE GOES HERE
  #
  ################################################
  def send(self):
    """ Send messages """
    # Send state [position, velocity]
    state = np.vstack((self.getPosition(), self.getVelocity(), self.vel_old, self.vel_older))
    self.vel_older = self.vel_old
    self.vel_old = self.getVelocity()
    for inbr in self.out_nbr:
      inbr.put(state) # send state to all neigbors

  def transition(self):      
    """ Update the state based on comms """
    # Check for failure
    if np.random.rand() <= self.fail_prob:
      self.fail_flag = True
    p = self.getPosition()
    if np.linalg.norm(self.getPosition()-self.goal) <= 2 and (-p[0]+19)-p[1] <= 0.05:
      self.land_flag = True
      self.finish = time.time()
      self.completion_time = self.finish - self.begin

    if self.fail_flag == False and self.land_flag == False:
        # Define summations for coherence, alignment, and collision avoidance
        tot_coh = np.array((0.0,0.0))
        tot_ali = np.array((0.0,0.0))
        tot_col = np.array((0.0,0.0))
        tot_cor = np.array((0.0,0.0))

        for inbr in self.in_nbr:
          # retrieve most recent message (timeout = 1s)
          data = inbr.get()
          if (not (data is None)):
            #print(data)
            #print(data[0])
            if (data[1]==data[2]).all() and (data[1]==data[3]).all() and not inbr.in_nbr.uid in self.sus_fail and inbr.in_nbr.land_flag == False:
              self.sus_fail.append(inbr.in_nbr.uid)
            nbr_pos = data[0]
            nbr_vel = data[1]

            # Check for collisions
            if np.linalg.norm(self.getPosition()-nbr_pos) <= 0.01:
              self.col_count += 1

            # Calculate cohesion, alignment, and collision terms
            tot_coh += nbr_pos - self.getPosition()
            tot_ali +=  nbr_vel - self.getVelocity()
            tot_col += self.getPosition() - nbr_pos
            #print(tot_coh)
            print("Node %d received data from %d " % (self.uid, inbr.in_nbr.uid))

            if inbr.in_nbr.uid in self.sus_fail:
              tot_cor += self.getPosition() - nbr_pos
        # Do some computations for control
        u_i = self.K_coh*tot_coh + self.K_ali*tot_ali + self.K_col*tot_col
        u_i += self.K_tar*(self.goal-self.getPosition()) + self.K_cor*tot_cor
        #print(u_i)

        self.velocity[0] += u_i[0]
        self.velocity[1] += u_i[1]
        self.velocity[0] = max(-self.max_speed,self.velocity[0])
        self.velocity[1] = max(-self.max_speed,self.velocity[1])
        self.velocity[0] = min(self.max_speed,self.velocity[0])
        self.velocity[1] = min(self.max_speed,self.velocity[1])

    elif self.fail_flag == False and self.land_flag == True:
        # Define summations for coherence, alignment, and collision avoidance
        tot_coh = np.array((0.0,0.0))
        tot_col = np.array((0.0,0.0))
        tot_cor = np.array((0.0,0.0))

        for inbr in self.in_nbr:
          # retrieve most recent message (timeout = 1s)
          data = inbr.get()
          if (not (data is None)):
            #print(data)
            #print(data[0])
            #if (data[1]==data[2]).all() and (data[1]==data[3]).all() and not inbr.in_nbr.uid in self.sus_fail and inbr.in_nbr.land_flag == False:
            #  self.sus_fail.append(inbr.in_nbr.uid)

            nbr_pos = data[0]
            nbr_vel = data[1]

            # Check for collisions
            if np.linalg.norm(self.getPosition()-nbr_pos) <= 0.01:
              self.col_count += 1

            # Calculate cohesion, alignment, and collision terms
            tot_coh += nbr_pos - self.getPosition()
            tot_col += self.getPosition() - nbr_pos
            #print(tot_coh)
            print("Node %d received data from %d " % (self.uid, inbr.in_nbr.uid))
            
            if inbr.in_nbr.uid in self.sus_fail:
              tot_cor += self.getPosition() - nbr_pos
        # Do some computation
        u_i = self.K_coh*tot_coh + self.K_col*tot_col
        u_i += self.K_cor*tot_cor
        #print(u_i)

        self.velocity *= self.K_break

        self.velocity[0] += u_i[0]
        self.velocity[1] += u_i[1]

        self.velocity[0] = max(-self.max_speed,self.velocity[0])
        self.velocity[1] = max(-self.max_speed,self.velocity[1])
        self.velocity[0] = min(self.max_speed,self.velocity[0])
        self.velocity[1] = min(self.max_speed,self.velocity[1])