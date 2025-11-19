import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB

class CVRP():
    '''
    class containing a three-dimensional loading capacitated vehicle routing problem (3L-CVRP)
    '''
    def __init__(self, name, nodes, links, vehicles, dimensions, boxes, demand, constraints):
        # Actual way to represent these inputs to be determined
        self.nodes = nodes
        self.depot = self.nodes[0]
        self.links = links
        self.demand = demand

        # Vehicle IDs and Vehicle Dimensions
        self.vehicles = vehicles
        self.dimensions = dimensions

        # Box sizes and IDs
        self.boxes = boxes
        self.boxID = list(boxes.keys())

        # Compute list of all possible length, width and height locations. To be reduced using form of the paper
        self.length = [i for i in range(0, dimensions["length"]-np.min([value[0] for value in self.boxes.values()])+1)]
        self.width = [i for i in range(0, dimensions["width"]-np.min([value[1] for value in self.boxes.values()])+1)]
        self.height = [i for i in range(0, dimensions["height"]-np.min([value[2] for value in self.boxes.values()])+1)]

        self.stages = [i+1 for i in range(len(nodes))]
        self.constraints = constraints

        self.model = gp.Model(name)

        # Create decision variables
        self.decision_variables()
        self.ObjectiveFunc()

        # Add constraints selectively, calls the function if it is enabled
        for key, value in self.constraints.items():
            if value:
                getattr(self, key)()

    def decision_variables(self):
        '''
        Create decision variables to be optimized, also encompasses constraint 6 and 11 which sets them to binary
        '''

        # Binary route decision variables \(d_{kl}^{tv}\)
        self.d = self.model.addVars(self.nodes, self.nodes, self.vehicles, self.stages,
                                    vtype=GRB.BINARY,
                                    name='d')

        # Binary loading decision variables \(a_{xyz}^{iktv}\)
        self.a = self.model.addVars(self.length, self.width, self.height, self.boxID, self.nodes[1:], self.vehicles, self.stages[:-1])

    def ObjectiveFunc(self):
        '''
        Takes link cost and routing decision variables and creates the objective function
        '''
        objective = gp.quicksum(self.links[i, j]["distance"] * self.d[i, j, v, t]
                                for i, j in self.links
                                for v in self.vehicles
                                for t in self.stages)

        self.model.setObjective(objective, GRB.MINIMIZE)

    def constraintTwo(self):
        '''
        Constraint two presented in the paper, ensures every customer is visited exactly once
        '''
        for k in self.nodes[1:]:
            self.model.addConstr(
                gp.quicksum(self.d[k, l, v, t]
                            for l in self.nodes
                            for v in self.vehicles
                            for t in self.stages
                )
                == 1,
                name=f"2|VisitOnce"
            )

    def constraintThree(self):
        '''
        Constraint three presented in the paper, ensures connectivity of each tour
        '''
        for k in self.nodes[1:]:
            self.model.addConstr(
                gp.quicksum(t * self.d[k, l, v, t]
                            for l in self.nodes
                            for v in self.vehicles
                            for t in self.stages[1:]
                )
                - gp.quicksum(t * self.d[p, k, v, t]
                            for p in self.nodes
                            for v in self.vehicles
                            for t in self.stages)
                == 1,
                name=f"3|Connectivity"
            )

    def constraintFour(self):
        '''
        Constraint four presented in paper, ensures vehicles leave the depot at most once at stage 1
        '''
        for v in self.vehicles:
            self.model.addConstr(
                gp.quicksum(self.d[1, l, v, 1]
                            for l in self.nodes[1:]
                )
                <= 1,
                name=f"4|LeaveDepotOnce"
            )

    def constraintFive(self):
        '''
        Constraint five presented in paper, ensures that if vehicle v travels from customer p to customer k at stage t,
        the vehicle travels from customer k to another customer I at stage t + 1
        '''
        for k in self.nodes[1:]:
            for t in self.stages[:-1]:
                for v in self.vehicles:
                    self.model.addConstr(
                        gp.quicksum(self.d[k, l, v, t+1]
                                    for l in self.nodes
                        )
                        - gp.quicksum(self.d[p, k, v, t]
                                      for p in self.nodes)
                        == 0,
                        name="5|CustomerToCustomer"
                    )

# Make results reproducable for the time being
np.random.seed(0)

# Depot (1) and customer nodes (2..., n)
nodes = [1, 2, 3, 4]

# Generate links from each node to each other node with random distances, might need to change to account for depot
links = {(i, j): {"distance": np.random.randint(10, 50) if i != j else 9999999} for i in nodes for j in nodes}

# Make it symmetric
for i, j in list(links.keys()):
    if i != j:
        links[(j, i)] = {"distance": links[(i, j)]["distance"]}

# Vehicle IDs [0, 1, ...]
vehicles = [0, 1]

# Dimensions of vehicles, identical for each vehicle
dimensions = {"length": 30, "width": 20, "height": 30}

# Idk what to do with the boxes yet. Will change.
boxes = {"box1": [1, 1, 1]}

# The above can be used for a more complicated situation. Below is some code that overwrites this all and generates a simple problem
# Extremely simple test problem
nodes = [1, 2, 3]
links = {(1, 1): {"distance": 9999},
         (1, 2): {"distance": 10},
         (2, 1): {"distance": 10},
         (1, 3): {"distance": 30},
         (3, 1): {"distance": 30},
         (2, 3): {"distance": 15},
         (3, 2): {"distance": 15},
         (2, 2): {"distance": 9999},
         (3, 3): {"distance": 9999}}

vehicles = [0]

constraints = {"constraintTwo": True,
               "constraintThree": True,
               "constraintFour": True,
               "constraintFive": True}

boxes = {1: [5, 10, 10],
         2: [5, 5, 5],
         3: [3, 2, 6]}

demand = {2: {1: 2, 2: 1, 3: 0},
                   3: {1: 0, 2: 0, 3: 1}}

problem = CVRP("3L_CVRP", nodes, links, vehicles, dimensions, boxes, demand, constraints=constraints)

problem.model.optimize()

if problem.model.status == GRB.OPTIMAL:
    print("\nActive decision variables (d[i,j,v,t] = 1):")
    for i, j, v, t in problem.d.keys():
        if problem.d[i, j, v, t].X > 0.5:  # X gives the value after optimization
            print(f"Vehicle {v} travels from node {i} to {j} at stage {t} | {links[i, j]}")