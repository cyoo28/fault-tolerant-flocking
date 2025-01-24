from locale import locale_alias
from Node import *
from Edge import *
from Graph import *

import numpy as np
import time

def generateRandomGraph(N):
  # Defining gains
  K_coh = 0.05
  K_ali = 0.02
  K_col = 0.053
  K_tar = 0.005
  K_cor = 0.05
  gains = [K_coh,K_ali,K_col,K_tar,K_cor]
  # Defining target location and fail rate
  fail_rate = 0.0005
  target = np.array([9.5,9.5])
  dist = np.linalg.norm(target-np.array([0.5,0.5]))
  G = Graph(None, dist)

  for inode in range(N):
    # randomly generate node states
    n = Node(inode, target, fail_rate,gains)
    # randomly spawn at location A (around [0.5,0.5])
    n.setPosition(np.random.rand(2))
    n.setVelocity(0.5 * np.random.rand(2) - 0.25)
    G.addNode(n)
  
    # add all-to-all edges
    for iedge in range(inode):
      G.addEdge(iedge, inode, 0)
      G.addEdge(inode, iedge, 0)
  
  return G


### MAIN
if __name__ == '__main__':

  # generate a random graph with 25 nodes
  G = generateRandomGraph(15)
  
  print("========== Starting now ==========")
  print("Close the figure to stop the simulation")
  G.run()             # start threads in nodes
  G.setupAnimation()  # set up plotting
  print("Sending stop signal.....")
  G.stop()            # send stop signal
  print("========== Terminated ==========")
  G.results()