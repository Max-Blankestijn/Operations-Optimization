import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB

class CVRP():
    '''
    class containing a three-dimensional loading capacitated vehicle routing problem (3L-CVRP)
    '''
    def __init__(self, name, nodes, links, vehicles, dimensions, boxes):
        # Actual way to represent these inputs to be determined
        self.nodes = nodes
        self.links = links
        self.vehicles = vehicles
        self.diensions = dimensions
        self.boxes = boxes

        self.model = gp.Model(name)


    def ObjectiveFunc(self):
        '''
        Takes link cost and routing decision variables and creates the objective function
        '''
        objective = gp.quicksum(links)

# Make results reproducable for the time being
np.random.seed(0)
nodes = [0, 1, 2, 3]

# Generate links from each node to each other node with random distances, might need to change to account for depot
links = {(i, j): {"distance": np.random.randint(10, 50)} for i in nodes for j in nodes if i != j}

# Vehicle IDs
vehicles = [0, 1]

# Dimensions of vehicles, identical for each vehicle
dimensions = {"length": 10, "width": 5, "height": 5}

# Idk what to do with the boxes yet. Will change.
boxes = {"box1": [1, 1, 1]}

problem = CVRP("3L_CVRP", nodes, links, vehicles, dimensions, boxes)