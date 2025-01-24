import numpy as np

class Edge:
  def __init__(self, in_nbr, out_nbr, cost):
    """ Constructor """
    self.in_nbr = in_nbr
    self.out_nbr = out_nbr
    self.cost = cost
    
    self.val = None
    
    self.commsradius = 1
    
  def __str__(self):
    """ Printing """
    return "Edge (%d, %d)" % (self.in_nbr.uid, self.out_nbr.uid)
    
  ################################################
  #
  # Implementing Communications
  #
  ################################################
  def getLength(self):
    return np.linalg.norm(self.in_nbr.position - self.out_nbr.position)

  def get(self):
    if (self.getLength() > self.commsradius):
      return None
    return self.val
    
  def put(self, x):
    if ( self.getLength() < self.commsradius ):
      self.val = x
