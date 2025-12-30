import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB
from helper import *

class CVRP():
    '''
    class containing a three-dimensional loading capacitated vehicle routing problem (3L-CVRP)
    '''
    def __init__(self, name, nodes, links, vehicles, dimensions, boxes, demand, constraints):

        # Define the nodes and demands (Depot 0, rest customer nodes)
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

        # Create lists with box lengths, widths, heights and the amount of a given box
        sizes_L = []
        sizes_W = []
        sizes_H = []
        counts = []

        for box_id, dims in self.boxes.items():
            sizes_L.append(dims[0])
            sizes_W.append(dims[1])
            sizes_H.append(dims[2])
            counts.append(sum(self.demand[box_id].values()))

        # Extract minimum dimensions (take transpose of dictionary values and take minimum)
        min_L, min_W, min_H = map(min, zip(*boxes.values()))

        # Create general set of possible positions
        self.xpos = reachable_positions(sizes_L, counts, self.dimensions["length"] - min_L)
        self.ypos = reachable_positions(sizes_W, counts, self.dimensions["width"] - min_W)
        self.zpos = reachable_positions(sizes_H, counts, self.dimensions["height"] - min_H)

        # Limit box i's positions to vehicle dimension minus box i's dimension to keep box inside
        self.xpos_lst = []
        self.ypos_lst = []
        self.zpos_lst = []

        for box_id, dims in self.boxes.items():
            self.xpos_lst.append({x for x in self.xpos if x <= self.dimensions["length"] - dims[0]})
            self.ypos_lst.append({y for y in self.ypos if y <= self.dimensions["width"] - dims[1]})
            self.zpos_lst.append({z for z in self.zpos if z <= self.dimensions["height"] - dims[2]})

        # Define time stages and active constraints
        self.stages = [i+1 for i in range(len(nodes))]
        self.constraints = constraints

        # Create the model
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
        self.a = self.model.addVars(self.xpos, self.ypos, self.zpos, self.boxID, self.nodes[1:], self.vehicles, self.stages[:-1],
                                    vtype=GRB.BINARY,
                                    name='a')

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
    
    def constraintSeven(self):
        return

    def constraintEight(self):
        '''
        Constraint eight presented in paper, ensures capacity of vehicles is not exceeded
        '''
        for v in self.vehicles:
            self.model.addConstr(
                gp.quicksum(self.boxes[i][0] * self.boxes[i][1] * self.boxes[i][2] * self.demand[i][k] * self.d[k, l, v, t]
                            for t in self.stages[1:]
                            for l in self.nodes
                            for k in self.nodes[1:]
                            for i in self.boxID)
                <= self.dimensions["length"] * self.dimensions["width"] * self.dimensions["height"],
                name="8|VehicleCapacity"
            )

    def constraintNine(self):
        '''
        Constraint nine presented in paper, ensures all boxes for customer k are unpacked when at that customer
        '''
        for k in self.nodes[1:]:
            for t in self.stages[:-1]:
                for v in self.vehicles:
                    self.model.addConstr(
                        gp.quicksum(self.a[x, y, z, i, k, v, t]
                                    for i in self.boxID
                                    for x in self.xpos
                                    for y in self.ypos
                                    for z in self.zpos)
                        ==
                        gp.quicksum(self.demand[i][k] * self.d[l, k, v, t]
                                    for i in self.boxID
                                    for l in self.nodes),
                        name="9|UnpackAll"
                    )

    def constraintTen(self):
        '''
        Constraint ten presented in paper, ensures boxes do not overlap. (Slows down model significantly)
        '''
        for x_prime in self.xpos:
            for y_prime in self.ypos:
                for z_prime in self.zpos:
                    for v in self.vehicles:
                        self.model.addConstr(
                            gp.quicksum(self.a[x, y, z, i, k, v, t]
                                        for i in self.boxID
                                        for k in self.nodes[1:]
                                        for t in self.stages[:-1]
                                        for x in self.xpos_lst[i-1]
                                        for y in self.ypos_lst[i-1]
                                        for z in self.zpos_lst[i-1]
                                        if x_prime - self.boxes[i][0] + 1 <= x <= x_prime
                                        if y_prime - self.boxes[i][1] + 1 <= y <= y_prime
                                        if z_prime - self.boxes[i][2] + 1 <= z <= z_prime
                            )
                            <=
                            1,
                            name="10|NoOverlapBoxes"
                        )

    def constraintEleven(self):
        '''
        Constraint eleven presented in paper, ensures the demand can be satisfied
        '''
        for i in self.boxID:
            for k in self.nodes[1:]:
                self.model.addConstr(
                    gp.quicksum(self.a[x, y, z, i, k, v, t]
                                for z in self.zpos_lst[i-1]
                                for y in self.ypos_lst[i-1]
                                for x in self.xpos_lst[i-1]
                                for v in self.vehicles
                                for t in self.stages[:-1])
                    ==
                    self.demand[i][k],
                    name="11|DemandSatisfiability"

                )

    # def constraintThirteen(self):
    #     for i in self.boxID:
    #         for k in self.nodes[1:]:
    #             for t in self.stages[:-1]:
    #                 for v in self.vehicles:
    #                     for x in self.xpos: # needs to become reduced version (X_t)
    #                         for y in self.ypos: #idem
    #                             for z in self.zpos: #idem, need to ad \{0}



if __name__ == "__main__":
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
    # vehicles = [0, 1]

    # Dimensions of vehicles, identical for each vehicle
    dimensions = {"length": 12, "width": 8, "height": 8}

    # The above can be used for a more complicated situation. Below is some code that overwrites this all and generates a simple problem
    # Extremely simple test problem
    nodes = [1, 2, 3, 4, 5]

    # Generate links from each node to each other node with random distances
    links = {(i, j): {"distance": np.random.randint(10, 50) if i != j else 9999999} for i in nodes for j in nodes}

    # Make it symmetric
    for i, j in list(links.keys()):
        if i != j:
            links[(j, i)] = {"distance": links[(i, j)]["distance"]}

    # links = {(1, 1): {"distance": 9999},
    #          (1, 2): {"distance": 10},
    #          (2, 1): {"distance": 10},
    #          (1, 3): {"distance": 30},
    #          (3, 1): {"distance": 30},
    #          (2, 3): {"distance": 15},
    #          (3, 2): {"distance": 15},
    #          (2, 2): {"distance": 9999},
    #          (3, 3): {"distance": 9999}}

    # Vehicle IDs
    # vehicles = [0, 1]
    vehicles = [0]

    # Active Constraints Dictionary from helper.py constraintGenerator function
    Nconstraints = 12
    constraints = constraintGenerator(range(1, Nconstraints+1))
    print('constraints', constraints)

    # Boxes
    boxes = {1: [2, 3, 4],
             2: [4, 2, 4],
             3: [3, 3, 3],
             4: [6, 2, 3]}

    # Customer Demand
    # key = box, keys in subdicts are nodes, values are number of boxes at each node
    # demand = {1: {2: 3, 3: 0, 4: 3, 5: 4},
    #           2: {2: 1, 3: 3, 4: 1, 5: 3},
    #           3: {2: 2, 3: 4, 4: 0, 5: 1},
    #           4: {2: 4, 3: 2, 4: 0, 5: 1}}
    
    demand = {1: {2: 1, 3: 1, 4: 0, 5: 0},
              2: {2: 0, 3: 1, 4: 1, 5: 0},
              3: {2: 0, 3: 0, 4: 1, 5: 1},
              4: {2: 1, 3: 0, 4: 0, 5: 1}}

    # Problem Creation
    problem = CVRP("3L_CVRP", nodes, links, vehicles, dimensions, boxes, demand, constraints=constraints)
    print("Model Created, starting optimization...")

    # Optimize model
    problem.model.optimize()
    print("Finished optimization")

    used_boxes1 = {1: [],
                  2: [],
                  3: [],
                  4: []}

    used_boxes2 = {1: [],
                  2: [],
                  3: [],
                  4: []}

    # Print out taken routes by vehicles
    if problem.model.status == GRB.OPTIMAL:
        print("\nActive decision variables (d[i,j,v,t] = 1):")
        for i, j, v, t in problem.d.keys():
            if problem.d[i, j, v, t].X > 0.5:  # X gives the value after optimization
                print(f"Vehicle {v} travels from node {i} to {j} at stage {t} | {links[i, j]}")
        for x, y, z, i, k, v, t in problem.a.keys():
            if problem.a[x, y, z, i, k, v, t].X > 0.5:
                if v == 0:
                    used_boxes1[i].append([x, y, z])
                if v == 1:
                    used_boxes2[i].append([x, y, z])
                print(f"Box of type {i} in vehicle {v} for customer {k} is at xyz: [{x},{y},{z}] at stage {t}")

    # Call the function
    plot_boxes_3d(used_boxes1, boxes, dimensions)
    # plot_boxes_3d(used_boxes2, boxes, dimensions)