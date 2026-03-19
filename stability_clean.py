import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors
import random
import copy
from time import time

import gurobipy as gp
from gurobipy import GRB
from helper import *
from model import CVRP
import sys

class StabilityAnalysisOneSyst():
    def __init__(self): #initiates the basic case from model.py
        self.nodes = [1, 2, 3, 4, 5]
        self.vehicles = [0, 1]
        self.dimensions = {"length": 12, "width": 8, "height": 8}
        
        self.links = create_links_from_coordinates({1:(100,100), 2:(102,160), 3:(20,178), 4:(72,35), 5:(152,28)}) # map coordinates
        self.boxes = {1: [2,3,4], 2:[4,2,4], 3:[4,3,3], 4:[6,2,3]} #box dimensions, right?

        self.links = self.randomLinks()

        # Customer Demand
        # key = box, keys in subdicts are nodes, values are number of boxes at each node
        self.demand = {1: {2:3, 3:0, 4:3, 5:4},
                2: {2:1, 3:3, 4:1, 5:3},
                3: {2:2, 3:4, 4:0, 5:1},
                4: {2:4, 3:2, 4:0, 5:1}}
        
        # Reach for removing boxes
        self.maximum_reach = [[self.boxes[i][0] for k in self.nodes[1:]] for i in self.boxes.keys()]

        # Fragility, set to 0 for no boxes on top of this box type, higher value means more load bearing capability
        self.sigma = [self.boxes[i][2] for i in self.boxes.keys()]

        # weight of box of type i
        density = 1
        self.p = [density * self.boxes[i][0] * self.boxes[i][1] * self.boxes[i][2] for i in self.boxes.keys()]
        self.genConstraints()

    def randomLinks(self, nNodes=None, xlim=300, ylim=300):
        points = {}
        node_ids = self.nodes if nNodes is None else list(range(1, nNodes + 1))
        for node_id in node_ids:
            newPoint = (random.randint(0, xlim), random.randint(0, ylim))
            points[node_id] = newPoint
        return create_links_from_coordinates(points)
        
    def randomDemands(self, rangeBox = [1,4]):
        for nodeOut in self.nodes:
            if nodeOut != self.nodes[-1]:
                for nodeIn in self.nodes:
                    if nodeIn != self.nodes[0]:
                        self.demand[nodeOut][nodeIn] = random.randint(rangeBox[0], rangeBox[1])

    def genConstraints(self, Nconstraints=19):
        # Active Constraints Dictionary from helper.py constraintGenerator function
        self.constraints = constraintGenerator(range(1, Nconstraints+1))
        #print('constraints', self.constraints)

    def optimize(self, printResults=False):
        # Problem Creation
        problem = CVRP("3L_CVRP", self.nodes, self.links, self.vehicles, self.dimensions, self.boxes, self.demand, self.maximum_reach, self.p, self.sigma, constraints=self.constraints)
        print("Model Created, starting optimization...")
        
        # Optimize model
        problem.model.params.TimeLimit = 120
        problem.model.optimize()
        print("Finished optimization")

        self.used_boxes1 = {1: [],
                    2: [],
                    3: [],
                    4: []}

        self.used_boxes2 = {1: [],
                    2: [],
                    3: [],
                    4: []}
        # Print out taken routes by vehicles
        if (printResults):
            if problem.model.status == GRB.OPTIMAL:
                print("\nActive decision variables (d[i,j,v,t] = 1):")
                for i, j, v, t in problem.d.keys():
                    if problem.d[i, j, v, t].X > 0.5:  # X gives the value after optimization
                        print(f"Vehicle {v} travels from node {i} to {j} at stage {t} | {self.links[i, j]}")
                for x, y, z, i, k, t, v in problem.a.keys():
                    if problem.a[x, y, z, i, k, t, v].X > 0.5:
                        if v == 0:
                            self.used_boxes1[i].append([x, y, z, k])
                        if v == 1:
                            self.used_boxes2[i].append([x, y, z, k])
                        print(f"Box of type {i} in vehicle {v} for customer {k} is at xyz: [{x},{y},{z}] at stage {t}")
            plot_boxes_3d(self.used_boxes1, self.boxes, self.dimensions)
            plot_boxes_3d(self.used_boxes2, self.boxes, self.dimensions)
        if (problem.model.SolCount > 0):
            return problem.model.ObjVal
        return -1
            
    def plot_network(self, node_size=300, cmap='tab10', linkDeltas=None, nodeDeltas=None):
        #plots the network with color coding signifying change
        fig, ax = plt.subplots(figsize=(6,6))
        nodes = list(self.nodes)
        n = len(nodes)
        if(linkDeltas==None):
            linkDeltas = np.zeros(len(self.nodes))
        if (nodeDeltas==None):
            nodeDeltas = np.zeros(n)
        angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
        node_positions = {node: (np.cos(angle), np.sin(angle)) for node, angle in zip(nodes, angles)}
        
        norm = colors.Normalize(vmin=min(linkDeltas), vmax=max(linkDeltas))
        cmap = cm.get_cmap('viridis')
        linkCols = [cmap(norm(i)) for i in linkDeltas]
        print(linkCols)
        
        norm = colors.Normalize(vmin=min(linkDeltas), vmax=max(linkDeltas))
        cmap = cm.get_cmap('viridis')
        linkCols = [cmap(norm(i)) for i in linkDeltas]
        
        seen = set()
        val = 0
        for (i,j), info in self.links.items():
            if i!=j:
                key = tuple(sorted((i,j)))
                if key in seen: continue
                else: seen.add(key)
                xi, yi = node_positions[i]
                xj, yj = node_positions[j]
                ax.plot([xi, xj], [yi, yj], color=linkCols[val], zorder=1)
                mx, my = (xi + xj) / 2, (yi + yj) / 2
                ax.text(mx, my, f'{linkDeltas[val]:.5f}', color='black', fontsize=8, ha='center', va='center', zorder=3)
                val += 1
        
        xs = [node_positions[n][0] for n in nodes]
        ys = [node_positions[n][1] for n in nodes]
        for idx, node in enumerate(nodes):
            ax.scatter(node_positions[node][0], node_positions[node][1], s=node_size, color='red', zorder=2) 
            ax.text(node_positions[node][0],node_positions[node][1], str(node), color='black', ha='center', zorder=4)
        ax.set_aspect('equal')
        ax.axis('off')
        return ax
    
    def link_v_distance(self, step=0.2, noSteps=1): #change this to get a graph for a node of distance v total cost
        #steps = np.linspace(1, 1+noSteps*step, num=noSteps)
        steps = [ 1, step+1 ]
        stabilities = []
        for step in steps:
            for (i,j), info in self.links.items():
                if (i != j):
                    info['distance'] *= step
                    self.links[(j,i)]['distance'] *= step
                    stabilities.append(self.optimize()) #currently this only works with with a single variation
                    info['distance'] /= step
                    self.links[(j,i)]['distance'] /= step                    
        return steps, stabilities/np.mean(stabilities)
    
    def vary_distance(self, step=0.1, noSteps=5):
        #steps = np.linspace(1 - step*noSteps/2, 1 + step*noSteps/2, noSteps)
        steps = np.linspace(1,2,2)
        costs = []
        tgtLinks = [(2,1), (1,2)]
        ogLinks = copy.deepcopy(self.links)
        for step in steps:
            self.links = copy.deepcopy(ogLinks)
            for link in tgtLinks:
                self.links[link]['distance'] *= step
                #print(self.links[link]['distance'])
            costs.append(self.optimize())
            #print(self.links[link]['distance'])
            self.links = ogLinks
            i = 0

        return steps, costs
    
    def vary_node_demands(self, step=0.1, noSteps = 5):
        #steps = np.linspace(1 - step*noSteps/2, 1 + step*noSteps/2, noSteps)
        steps = [0.75, 1., 1.25, 1.5]
        print(steps)
        stabilities = [ ]
        ogDemand = self.demand
        i = 0
        for step in steps:
            stabLine = [ 0 ]
            for node in self.demand:
                for item in self.demand[node]:
                    self.demand[node][item] *= step
                    self.demand[node][item] = int(self.demand[node][item])
                stabLine.append(self.optimize())
                self.demand = ogDemand
            stabilities.append([stabLine])
           #self.status(i, noSteps)
        return steps, stabilities

    def vary_demands(self, start = 0, end=5, noSteps=5):
        steps = np.linspace(start, end, noSteps)
        steps = [ 0.2, 0.4, 0.6, 0.8, 1]
        costs = [ ]
        tgtNode = 4
        ogVal = copy.deepcopy(self.demand)
        for step in steps:
            self.demand = copy.deepcopy(ogVal)
            for item in self.demand[tgtNode]:
                self.demand[tgtNode][item] = int(self.demand[tgtNode][item] * step)
            costs.append(self.optimize())
            self.demand = ogVal
        return steps, costs
    
    def vary_all_demands(self, steps=[1.1]):
        ogVal = copy.deepcopy(self.demand)
        costs = []
        self.demand = copy.deepcopy(ogVal)
        for step in steps:
            for tgtNode in self.nodes:
                for item in self.demand[tgtNode]:
                    self.demand[tgtNode][item] = int(self.demand[tgtNode][item] * step)
        costs.append(self.optimize())
        self.demand = ogVal
        return costs
            
    def constraintCost(self, totalN = 19, plot=False):
        costs = []
        steps = np.zeros([1,19])
        for i in range(totalN):
            self.genConstraints(i)
            print(i)
            costs.append(self.optimize())
        if (plot):
            self.plotGraph(costs)
        return steps, costs
    
    def status(done, total):
        print(str(done) + " out of " + str(total) + " done, " + str(done/total * 100) + "%")

    def vary_reach(self, step = 2, noSteps = 1, demand = 0.5):
        #steps = np.linspace(1-step*noSteps/2, 1-step*noSteps/2, noSteps)
        steps = [ 0.2, 0.4, 0.6, 0.8] #the way to actually do this is go from 1 to such that maximum would reach across the full vehicle, right?
        costs = []
        for tgtNode in self.demand:
            for item in self.demand[tgtNode]:
                self.demand[tgtNode][item] = int(self.demand[tgtNode][item] * demand)       
        for step in steps:
            ogReach = self.maximum_reach
            self.maximum_reach = [[self.boxes[i][0]*step for k in self.nodes[1:]] for i in self.boxes.keys()]
            costs.append(self.optimize())
            self.maximum_reach = ogReach
        return steps, costs
    
    def vary_sigma(self, step = 2, noSteps = 1, reachability=1, demand=0.2):
        #steps = np.linspace(1-step*noSteps/2, 1-step*noSteps/2, noSteps)
        steps = [ 2] #the way to actually do this is go from 1 to such that maximum would reach across the full vehicle, right?
        costs = []
        if (reachability != 1 or demand != 1):
            ogDemand = self.demand
            self.demand = copy.deepcopy(ogDemand)
            ogReach = self.maximum_reach
            self.maximum_reach = copy.deepcopy(ogReach)
            self.maximum_reach = [[self.boxes[i][0]*reachability for k in self.nodes[1:]] for i in self.boxes.keys()]
            for tgtNode in self.demand:
                for item in self.demand[tgtNode]:
                    self.demand[tgtNode][item] = int(self.demand[tgtNode][item] * demand)      
                    print(self.demand[tgtNode][item])  
        for step in steps:
            ogSigma = self.sigma
            self.sigma = [self.boxes[i][2]*step for i in self.boxes.keys()]
            costs.append(self.optimize())
            self.sigma = ogSigma
            #plot_boxes_3d(self.used_boxes1, self.boxes, self.dimensions)
            #plot_boxes_3d(self.used_boxes2, self.boxes, self.dimensions)
        
        if (reachability != 1 and demand != 1):
            self.demand = copy.deepcopy(ogDemand)
            self.maximum_reach = copy.deepcopy(ogReach)
        return steps, costs

    def bisect(self, fun, up=10, lo=0, tgt=0.1):
        upInit = fun(self, up)
        loInit = fun(self, lo)
        while(up-lo > tgt):
            value = fun(self, (up+lo)/2)
            #if value ==
    
    def multiVehicle(self, demand=0.2):
        if (demand != 1):
            ogDemand = self.demand
            self.demand = copy.deepcopy(ogDemand)            
        options = [[0], [0,1], [0,1,2], [0,1,2,3]]
        Ns = [1,2,3]
        costs = []
        for opt in options:
            self.vehicles = opt
            costs.append(self.optimize())
        return Ns, costs
    
    def addNodes(self, nMin = 1, nMax=5):
        self.nodes = []
        costs = []
        steps = []
        for i in range(nMin - 1):
            self.nodes.append(i)
        for i in range(nMin, nMax):
            self.nodes.append(i)
            self.randomLinks()
            costs.append(self.optimize())
            steps.append[i]
        return steps, costs    
        
def marg(costs):
    margs = [ costs[0] ]
    for i in range(len(costs)-1):
        margs.append(costs[i+1] - costs[i])
    return margs

if __name__ == "__main__":
    start_time = time()
    Nruns = 10
    totCosts = []
    totSteps = []
    for i in range(Nruns):
        sys = StabilityAnalysisOneSyst()
        #sys.randomDemands()
        steps, costs = sys.constraintCost()
        #steps, costs = sys.vary_demands()
        totSteps.append(steps)
        totCosts.append(costs)
        #sys.randomDemands()
    print(totSteps)
    print(totCosts)
    #print(marg(costs))
    #print(sys.vary_sigma(reachability=0.5, demand = 0.5))
    print("Run time: " + str(time() - start_time))
    plt.plot(steps, costs)
    #plt.plot(steps, marg(costs))
    plt.show()