import unittest
import numpy as np
import gurobipy as gp
from gurobipy import GRB

from model import CVRP
from helper import make_links

class TestCVRP(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """
        Create multiple test setups to be reused across all constraint tests.
        Each setup is a dict, all tests should work for every possible setup.
        This is NOT for making deterministic tests we can hand verify, I'll do that in a seperate file I think
        """

        cls.test_inputs = []
        cls.solved_models = []

        # Different configurations, this list will become quite big I think
        test_scenarios = [
            {"name": "Small_1veh_3nodes", "nodes": [1, 2, 3], "vehicles": [0]},
            {"name": "Small_2veh_3nodes", "nodes": [1, 2, 3], "vehicles": [0, 1]},
            {"name": "Medium_1veh_6nodes", "nodes": list(range(1, 7)), "vehicles": [0]},
            {"name": "Medium_2veh_6nodes", "nodes": list(range(1, 7)), "vehicles": [0, 1]},
            # {"name": "Large_3veh_20nodes", "nodes": list(range(1, 21)), "vehicles": [0, 1, 2]},
        ]

        for scenario in test_scenarios:
            nodes = scenario["nodes"]
            links = make_links(nodes)

            # Will be added in the dict of test_scenarios once they actually do stuff
            dimensions = {"length": 12, "width": 8, "height": 8}
            boxes = {1: [2, 3, 4],
             2: [4, 2, 4],
             3: [4, 3, 3],
             4: [6, 2, 3]}
            
            demand = {1: {2: 3, 3: 0, 4: 3, 5: 4, 6: 0},
              2: {2: 1, 3: 3, 4: 1, 5: 3, 6: 0},
              3: {2: 2, 3: 4, 4: 0, 5: 1, 6: 0},
              4: {2: 4, 3: 2, 4: 0, 5: 1, 6: 0}
            }
            
            case = {
                "name": scenario["name"],
                "nodes": nodes,
                "links": links,
                "vehicles": scenario["vehicles"],
                "dimensions": dimensions,
                "boxes": boxes,
                "demand": demand
            }

            cls.test_inputs.append(case)
    

            testmodel = CVRP(
                case["name"],
                case["nodes"],
                case["links"],
                case["vehicles"],
                case["dimensions"],
                case["boxes"],
                case["demand"],
                constraints={
                "constraintTwo": True,
                "constraintThree": True,
                "constraintFour": True,
                "constraintEight": True,
                }
            )
        
            testmodel.model.setParam("OutputFlag", 0) # Removes all printstuff
            testmodel.model.optimize()

            #Add a list of models to be tested
            cls.solved_models.append((case["name"], testmodel))


    # def test_constraint_two(self):
    #     for case_name, model in self.solved_models:
    #         with self.subTest(case=case_name):
    #             if model.model.status == GRB.OPTIMAL:
    #                 for k in model.nodes[1:]:
    #                     self.assertAlmostEqual(
    #                         sum(model.d[k, l, v, t].X
    #                                     for l in model.nodes
    #                                     for v in model.vehicles
    #                                     for t in model.stages
    #                         )
    #                         , 1.0
    #                         ,"Fail constraint two")
    
    # def test_constraint_three(self):
    #     for case_name, model in self.solved_models:
    #         with self.subTest(case=case_name):
    #             if model.model.status == GRB.OPTIMAL:
    #                 for k in model.nodes[1:]:
    #                     self.assertAlmostEqual(
    #                         sum(t * model.d[k, l, v, t].X
    #                             for l in model.nodes
    #                             for v in model.vehicles
    #                             for t in model.stages[1:]
    #                         )
    #                         - sum(t * model.d[p, k, v, t].X
    #                             for p in model.nodes
    #                             for v in model.vehicles
    #                             for t in model.stages)
    #                         , 1.0
    #                         ,"Fail Constraint Three"
    #                     )
     
    # def test_constraint_four(self):
    #     for case_name, model in self.solved_models:
    #         with self.subTest(case=case_name):
    #             if model.model.status == GRB.OPTIMAL:
    #                 for v in model.vehicles:
    #                     self.assertGreaterEqual(
    #                         1.0,
    #                         sum(model.d[1, l, v, 1].X
    #                                     for l in model.nodes[1:]
    #                         ),
    #                         "Fail Constraint Four"
    #                     )

    def test_constraint_eight(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                if model.model.status == GRB.OPTIMAL:
                    for v in model.vehicles:
                        print(model.dimensions["length"] * model.dimensions["width"] * model.dimensions["height"])
                        print(model.dimensions["length"] * model.dimensions["width"] * model.dimensions["height"],
                            sum(model.boxes[i][0] * model.boxes[i][1] * model.boxes[i][2] * model.demand[i][k] * model.d[k, l, v, t].X
                                for t in model.stages[1:]
                                for l in model.nodes
                                for k in model.nodes[1:]
                                for i in model.boxID))
                        self.assertGreaterEqual(
                            model.dimensions["length"] * model.dimensions["width"] * model.dimensions["height"],
                            sum(model.boxes[i][0] * model.boxes[i][1] * model.boxes[i][2] * model.demand[i][k] * model.d[k, l, v, t].X
                                for t in model.stages[1:]
                                for l in model.nodes
                                for k in model.nodes[1:]
                                for i in model.boxID)
                        )







if __name__ == "__main__":
    unittest.main()