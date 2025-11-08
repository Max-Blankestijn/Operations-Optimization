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
        self.dimensions = dimensions
        self.boxes = boxes
        self.stages = [i for i in range(len(nodes))]

        self.model = gp.Model(name)

        # Create decision variables
        self.decision_variables()
        self.ObjectiveFunc()

    def decision_variables(self):
        '''
        Create decision variables to be optimized
        '''

        # Binary decision variables \(d_{kl}^{tv}\)
        self.x = self.model.addVars(self.nodes, self.nodes, self.vehicles, self.stages,
                                    vtype=GRB.BINARY,
                                    name='d')
    def ObjectiveFunc(self):
        '''
        Takes link cost and routing decision variables and creates the objective function
        '''
        objective = gp.quicksum(self.links[i, j]["distance"] * self.x[i, j, v, t]
                                for i, j in self.links
                                for v in self.vehicles
                                for t in self.stages)

        self.model.setObjective(objective, GRB.MINIMIZE)

# Make results reproducable for the time being
np.random.seed(0)

# Depot (1) and customer nodes (2..., n)
nodes = [1, 2, 3, 4]

# Generate links from each node to each other node with random distances, might need to change to account for depot
# Infinity if link goes to itself in accordance with the paper
links = {(i, j): {"distance": np.random.randint(10, 50) if i != j else 9999999} for i in nodes for j in nodes}

# Vehicle IDs
vehicles = [0, 1]

# Dimensions of vehicles, identical for each vehicle
dimensions = {"length": 10, "width": 5, "height": 5}

# Idk what to do with the boxes yet. Will change.
boxes = {"box1": [1, 1, 1]}

problem = CVRP("3L_CVRP", nodes, links, vehicles, dimensions, boxes)

problem.model.optimize()

if problem.model.status == GRB.OPTIMAL:
    print("\nActive decision variables (x[i,j,v,t] = 1):")
    for i, j, v, t in problem.x.keys():
        if problem.x[i, j, v, t].X > 0.5:  # X gives the value after optimization
            print(f"Vehicle {v} travels from node {i} to {j} at stage {t}")