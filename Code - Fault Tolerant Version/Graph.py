from Node import *
from Edge import *

from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np

class Graph:
  def __init__(self, filename = None, dist=0):
    """ Constructor """
    self.Nv = 0
    self.V = []
    self.E = []
    self.root = None
    # define some extra metrics
    self.dist = dist
    self.spread = []
    # for plotting
    self.animatedt = 100 # milliseconds
    self.fig = plt.figure()
    self.ax = plt.axes(xlim=(-1.0, 11.0), ylim=(-1.0, 11.0))
    self.ax.set_aspect('equal', 'box')
    self.pts, = self.ax.plot([], [], 'b.')
    self.anim = None
    
    # for reading in graphs if they come from a file
    if not(filename is None):
      # read the graph from a file
      with open(filename) as f:
        # nodes
        line = f.readline()
        self.Nv = int(line);
        for inode in range(self.Nv):
          self.addNode(Node(inode))

        # edges      
        line = f.readline()
        while line:
          data = line.split()
        
          in_nbr = int(data[0])
          out_nbr = int(data[1])
          cost = float(data[2])
        
          self.addEdge(in_nbr, out_nbr, cost)
        
          line = f.readline()
      
      f.close()
    
  def __str__(self):
    """ Printing """
    return "Graph: %d nodes, %d edges" % (self.Nv, len(self.E))
    
  ################################################
  #
  # Modify the graph
  #
  ################################################

  def addNode(self, n):
    """ Add a node to the graph """
    self.V.append(n)
    self.Nv += 1
    
  def addEdge(self, i, o, c):
    """ Add an edge between two nodes """
    e = Edge(self.V[i], self.V[o], c)
    self.V[i].addOutgoing(e)
    self.V[o].addIncoming(e)
    self.E.append(e)
    
  ################################################
  #
  # Start and Stop computations
  #
  ################################################

  def run(self):
    """ Run the alg on all of the nodes """
    # Start running the threads
    for i in range(self.Nv):
      self.V[i].start()

  def stop(self):
    """ Send a stop signal """
    # Send a stop signal
    for i in range(self.Nv):
      self.V[i].terminate()
    # Wait until all the nodes are done
    for i in range(self.Nv):
      self.V[i].join()
      
  ################################################
  #
  # Animation helpers
  #
  ################################################

  def gatherNodeLocations(self):
    """ Collect state information from all the nodes """
    x = []; y = [];
    p = []
    for i in range(self.Nv):
      x.append(self.V[i].position[0])
      y.append(self.V[i].position[1])
      if self.V[i].fail_flag == False:
        p.append(self.V[i].position)
    tot_spread = 0
    avg_spread = 0
    for i in p:
      for j in p:
        tot_spread += np.linalg.norm(np.array(i)-np.array(j))
      avg_spread += tot_spread/(len(p)-1)
    self.spread.append(avg_spread/len(p))
    return x,y
      
  def setupAnimation(self):
    """ Initialize the animation """
    self.anim = animation.FuncAnimation(self.fig, self.animate, interval=self.animatedt, blit=False)
    
    plt.show()
    
  def animate(self, i):
    """ Animation helper function """
    x,y = self.gatherNodeLocations()
    self.pts.set_data(x, y)
    
    return self.pts, 

# Display relevant data
  def results(self):
    # Average stats across all nodes
    tot_time = 0
    tot_col = 0
    tot_fail = 0
    tot_sus = 0

    for boid in self.V:   
      if boid.fail_flag == True:
        tot_fail += 1
      else:
        tot_time += boid.completion_time
        tot_sus += len(boid.sus_fail)
      tot_col += boid.col_count
    
    avg_time = tot_time/self.Nv
    avg_sus = tot_sus/self.Nv

    # Display stats
    # Number of boids
    print("Number of boids: {}".format(self.Nv))
    # Flocking distance
    print("Flight Path Distance: {:.2f} units".format(self.dist))
    # Completion time
    print("Flight Duration: {:.2f} s".format(avg_time))
    # Number of collisions
    print("Number of Collisions: {}".format(tot_col))
    # Number of failed agents
    print("Number of actual failures: {}".format(tot_fail))
    # Number of suspected agents
    print("Number of suspected failures: {}".format(avg_sus))

    # Plot 
    # spread vs time
    #print("{}".format(spread.shape))
    t = np.arange(0, avg_time, avg_time/len(self.spread))
    plt.plot(t, self.spread)
    plt.title("Flock Spread During Flight Time")
    plt.xlabel("Time")
    plt.ylabel("Average Flock Spread")
    plt.show()
