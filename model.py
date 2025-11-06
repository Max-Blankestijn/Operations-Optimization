import numpy as np
import scipy as sp
import matplotlib.pyplot as plt


import gurobipy as gp
from gurobipy import GRB

# Create a new model
model = gp.Model("simple_linear_program")

# Create variables
x = model.addVar(name="x", lb=0)  # x >= 0
y = model.addVar(name="y", lb=0)  # y >= 0

# Set objective: maximize 3x + 4y
model.setObjective(3 * x + 4 * y, GRB.MAXIMIZE)

# Add constraints
model.addConstr(2 * x + y <= 8, name="c1")
model.addConstr(x + 2 * y <= 8, name="c2")

# Optimize the model
model.optimize()

# Display results
if model.status == GRB.OPTIMAL:
    print(f"Optimal objective value: {model.objVal}")
    for v in model.getVars():
        print(f"{v.varName} = {v.x}")
