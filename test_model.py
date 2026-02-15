import unittest
import numpy as np
import gurobipy as gp
from gurobipy import GRB

from model import CVRP
from helper import make_links, make_boxes, make_demand, get_model, create_links_from_coordinates

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
        cls.infeasible_models = []

        # Different configurations
        # If left unspecified: node_amount = 6, vehicle_amount = 2, box_amount = 3 (number of types of boxes)
        cls.test_scenarios = [
            {"name": "Small_1veh_3nodes_1b", "node_amount": 3, "vehicle_amount": 1, "box_amount": 1},
            {"name": "Small_1veh_3nodes_2b", "node_amount": 3, "vehicle_amount": 1, "box_amount": 2},
            {"name": "Small_1veh_3nodes_3b", "node_amount": 3, "vehicle_amount": 1, "box_amount": 3},
            {"name": "Small_1veh_3nodes_4b", "node_amount": 3, "vehicle_amount": 1, "box_amount": 4},
            {"name": "Small_1veh_3nodes_5b", "node_amount": 3, "vehicle_amount": 1, "box_amount": 5},
            {"name": "Small_2veh_3nodes_1b", "node_amount": 3, "box_amount": 1},
            {"name": "Small_2veh_3nodes_2b", "node_amount": 3, "box_amount": 2},
            {"name": "Medium_1veh_6nodes_2b"  , "vehicles": 1, "box_amount": 2},
            {"name": "Medium_2veh_6nodes_2b"  , "vehicles": 2, "box_amount": 2},
            {"name": "Medium_2veh_6nodes_3b"  , "vehicles": 2, "box_amount": 3},

            #More vehicles than boxes
            {"name": "Medium_3veh_3nodes", "node_amount": 3, "vehicles": 3, "box_amount": 1,
              "demand": {1: {2:1, 3:1}}},
            {"name": "Medium_3veh_3nodes", "node_amount": 3, "vehicles": 3, "box_amount": 2,
              "demand": {1: {2:1, 3:0},
                         2: {2:0, 3:1}}},
            
            #Tests to show that boxes fit exactly
            {"name": "PerfectFit1"         , "node_amount": 2, "vehicle_amount": 1, "box_amount": 1,
            "vehicle_size": {"length": 2, "width": 3, "height": 4}, "demand":{1: {2: 1}}, "boxes": {1: [2,3,4]}},
            {"name": "PerfectFit2"         , "node_amount": 2, "vehicle_amount": 1, "box_amount": 1,
            "vehicle_size": {"length": 2, "width": 6, "height": 4}, "demand":{1: {2: 2}}, "boxes": {1: [2,3,4]}},
            {"name": "PerfectFit3"         , "node_amount": 2, "vehicle_amount": 1, "box_amount": 1,
            "vehicle_size": {"length": 4, "width": 3, "height": 4}, "demand":{1: {2: 2}}, "boxes": {1: [2,3,4]}},
            {"name": "PerfectFit4"         , "node_amount": 2, "vehicle_amount": 2, "box_amount": 1,
            "vehicle_size": {"length": 2, "width": 3, "height": 4}, "demand":{1: {2: 1}}, "boxes": {1: [2,3,4]}},

            #Tests to show that boxes don't fit, these scenarios should be infeasible
            {"name": "NotSoPerfectFit1"         , "node_amount": 2, "vehicle_amount": 1, "box_amount": 1,
            "vehicle_size": {"length": 1.9, "width": 3, "height": 4}, "demand":{1: {2: 1}}, "boxes": {1: [2,3,4]}},
            {"name": "NotSoPerfectFit2"         , "node_amount": 2, "vehicle_amount": 1, "box_amount": 1,
            "vehicle_size": {"length": 2, "width": 2.9, "height": 4}, "demand":{1: {2: 1}}, "boxes": {1: [2,3,4]}},
            {"name": "NotSoPerfectFit3"         , "node_amount": 2, "vehicle_amount": 1, "box_amount": 1,
            "vehicle_size": {"length": 2, "width": 3, "height": 3.9}, "demand":{1: {2: 1}}, "boxes": {1: [2,3,4]}},
            {"name": "NotSoPerfectFit4"         , "node_amount": 2, "vehicle_amount": 1, "box_amount": 1,
            "vehicle_size": {"length": 2, "width": 5.9, "height": 4}, "demand":{1: {2: 2}}, "boxes": {1: [2,3,4]}},
            # {"name": "Large_3veh_20nodes", "nodes": list(range(1, 21)), "vehicles": [0, 1, 2], "box_amount": 6},

            #Hand verifiable tests
            # These tests have link combinations which have a very obvious solution (i.e straight line)
            # 1 - 2 - 3 - 4 - 5
            # 1 is the start node, between each node is a distance of 5, so between 1 - 2 = 5, 1 - 3 = 10, 1 - 4 = 15, 2 - 3 = 5 etc.
            {"name": "StaightLine_1veh", "node_amount": 5, "vehicle_amount": 1, "box_amount": 1,
             "demand": {1: {2:1, 3:1, 4:1, 5:1}}, 
             "links": create_links_from_coordinates({1: (0,0), 2: (0,5), 3:(10, 5), 4: (10, -5), 5:(0,-5)}),
             "vehicle_size": {"length":10, "width": 2, "height": 2},
             "boxes": {1: [2,2,2]}
             },

            #Same case as before, but with additional box at node 3, only 2 logical ways of packing
             {"name": "StaightLine_1veh2box", "node_amount": 5, "vehicle_amount": 1, "box_amount": 1,
             "demand": {1: {2:1, 3:1, 4:1, 5:1},
                        2: {2:0, 3:1, 4:0, 5:0}}, 
             "links": create_links_from_coordinates({1: (0,0), 2: (0,5), 3:(10, 5), 4: (10, -5), 5:(0,-5)}),
             "vehicle_size": {"length":9, "width": 2, "height": 2},
             "boxes": {1: [2,2,2],
                       2: [1,2,2]}
             },
        ]

        for scenario in cls.test_scenarios:
            #For each possible changed variable there is a "standard" amount if it is not specified in the scenario

            #Amount of Nodes (normal = 6)
            node_amount = scenario.get("node_amount", 6)
            nodes = list(range(1, node_amount+1))
            
            #Links between Nodes
            links = scenario.get("links", make_links(nodes))

            #Amount of Vehicles (normal = 2)
            vehicle_amount = scenario.get("vehicle_amount", 2)
            vehicles = list(range(0, vehicle_amount))
            
            #Number of types of boxes (normal = 3)
            box_amount = scenario.get("box_amount", 3)
            boxes = scenario.get("boxes", make_boxes(box_amount))

            # Dimension of vehicle
            dimensions = scenario.get("vehicle_size", {"length": 12, "width": 8, "height": 8})

            #First number is box, numbers in sub dict are the nodes
            demand = scenario.get("demand", make_demand(box_amount, node_amount))

            # Reach for removing boxes
            maximum_reach = scenario.get("maximum_reach", [[boxes[i][0] for k in nodes[1:]] for i in boxes.keys()])

            # Fragility, set to 0 for no boxes on top of this box type, higher value means more load bearing capability
            sigma = scenario.get("sigma", [9999999 for i in boxes.keys()])

            # weight of box of type i
            density = 1
            p = [density * boxes[i][0] * boxes[i][1] * boxes[i][2] for i in boxes.keys()]
            
            case = {
                "name": scenario["name"],
                "nodes": nodes,
                "links": links,
                "vehicles": vehicles,
                "dimensions": dimensions,
                "boxes": boxes,
                "demand": demand,
                "maximum_reach": maximum_reach,
                "sigma": sigma,
                "p": p
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
                case["maximum_reach"],
                case["sigma"],
                case["p"],
                constraints={
                "constraintTwo": True,
                "constraintThree": True,
                "constraintFour": True,
                "constraintFive": True,
                "constraintEight": True,
                "constraintNine": True,
                "constraintTen": True,
                "constraintEleven": True,
                "constraintThirteen": False, #This constraint makes the code hella slow
                "constraintFourteen": True,
                "constraintFifteen": True,
                "constraintSixteen": True,
                "constraintSeventeen": True,
                "constraintEighteen": True,
                }
            )
        
            testmodel.model.setParam("OutputFlag", 0) # Removes all printstuff
            testmodel.model.optimize()

            #Add a list of models to be tested, that are solved
            if testmodel.model.status == 2:
                cls.solved_models.append((case["name"], testmodel))
            
            if testmodel.model.status == 3:
                print(case["name"], "is infeasible")
                cls.infeasible_models.append((case["name"], testmodel))


    def test_constraint_two(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for k in model.nodes[1:]:
                    self.assertAlmostEqual(
                        sum(model.d[k, l, v, t].X
                                    for l in model.nodes
                                    for v in model.vehicles
                                    for t in model.stages
                        )
                        , 1.0
                        ,"Fail constraint two")
    
    def test_constraint_three(self):
        """
        Docstring for test_constraint_three
        
        
        """
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for k in model.nodes[1:]:
                    self.assertAlmostEqual(
                        sum(t * model.d[k, l, v, t].X
                            for l in model.nodes
                            for v in model.vehicles
                            for t in model.stages[1:]
                        )
                        - sum(t * model.d[p, k, v, t].X
                            for p in model.nodes
                            for v in model.vehicles
                            for t in model.stages)
                        , 1.0
                        ,"Fail Constraint Three"
                    )
     
    def test_constraint_four(self):
        """
        Docstring for test_constraint_four
        
        Verifies that vehicles leave depot exactly once, at stage one
        """
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for v in model.vehicles:
                    self.assertGreaterEqual(
                        1.0,
                        sum(model.d[1, l, v, 1].X
                                    for l in model.nodes[1:]
                        ),
                        "Fail Constraint Four"
                    )

    def test_constraint_five(self):
        """
        Docstring for test_constraint_five
        
        Verifies that a customer after having traveled to node k at time t, travels from node k at time t+1
        """

        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for k in model.nodes[1:]:
                    for t in model.stages[:-1]:
                        for v in model.vehicles:
                            self.assertAlmostEqual(
                                sum(model.d[k, l, v, t+1].X
                                    for l in model.nodes
                                )
                                - sum(model.d[p, k, v, t].X
                                    for p in model.nodes),
                                0.0,
                                "Fail Constraint Five"
                            )

    def test_constraint_eight(self):
        """
        Docstring for test_constraint_eight
        
        Verifies that the resulting boxplacement does not have volume exceeding the volume of the vehicle
        """
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for v in model.vehicles:
                    self.assertGreaterEqual(
                        model.dimensions["length"] * model.dimensions["width"] * model.dimensions["height"],
                        sum(model.boxes[i][0] * model.boxes[i][1] * model.boxes[i][2] * model.demand[i][k] * model.d[k, l, v, t].X
                            for t in model.stages[1:]
                            for l in model.nodes
                            for k in model.nodes[1:]
                            for i in model.boxID),
                        "Fail Constraint Eight"
                    )

    def test_constraint_nine(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for k in model.nodes[1:]:
                    for t in model.stages[:-1]:
                        for v in model.vehicles:
                            self.assertAlmostEqual(
                                sum(model.a[x, y, z, i, k, t, v].X
                                    for i in model.boxID
                                    for x in model.xpos
                                    for y in model.ypos
                                    for z in model.zpos),
                                sum(model.demand[i][k] * model.d[l, k, v, t].X
                                    for i in model.boxID
                                    for l in model.nodes),
                                "Fail Constraint Nine"
                            )
    
    def test_constraint_ten(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for x_prime in model.xpos:
                    for y_prime in model.ypos:
                        for z_prime in model.zpos:
                            for v in model.vehicles:
                                self.assertGreaterEqual(
                                    1.0,
                                    sum(model.a[x,y,z,i,k,t,v].X
                                        for i in model.boxID
                                        for k in model.nodes[1:]
                                        for t in model.stages[:-1]
                                        for x in model.xpos_lst[i-1] if x_prime - model.boxes[i][0] + 1 <= x <= x_prime
                                        for y in model.ypos_lst[i-1] if y_prime - model.boxes[i][1] + 1 <= y <= y_prime
                                        for z in model.zpos_lst[i-1] if z_prime - model.boxes[i][2] + 1 <= z <= z_prime
                                    ),
                                    "Fail Constraint 10"
                                )
    
    def test_constraint_eleven(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for i in model.boxID:
                    for k in model.nodes[1:]:
                        self.assertAlmostEqual(
                            sum(model.a[x, y, z, i, k, t, v].X
                                        for z in model.zpos_lst[i-1]
                                        for y in model.ypos_lst[i-1]
                                        for x in model.xpos_lst[i-1]
                                        for v in model.vehicles
                                        for t in model.stages[:-1]),
                            model.demand[i][k],
                            "Fail Constraint 11"
                        )
    
    # def test_constraint_thirteen(self):
    #     for case_name, model in self.solved_models:
    #         with self.subTest(case=case_name):
    #             for i in model.boxID:
    #                 for k in model.nodes[1:]:
    #                     for t in model.stages[:-1]:
    #                         for v in model.vehicles:
    #                             for x in model.xpos_lst[i-1]:
    #                                 for y in model.ypos_lst[i-1]:
    #                                     for z in model.zpos_lst[i-1][1:]:
    #                                         self.assertGreaterEqual(
    #                                             sum((min(x + model.boxes[i][0], x_pp + model.boxes[j][0]) - max(x, x_pp)) * \
    #                                                         (min(y + model.boxes[i][1], y_pp + model.boxes[j][1]) - max(y, y_pp)) * \
    #                                                         model.a[x_pp, y_pp, z-model.boxes[j][2], j, l, u, v].X
    #                                                         for j in model.boxID if z - model.boxes[j][2] >= 0 and z - model.boxes[j][2] in model.zpos
    #                                                         for l in model.nodes[1:]
    #                                                         for u in model.nodes[:-1] if u >= t
    #                                                         for x_pp in model.xpos_lst[j-1] if x - model.boxes[j][0] + 1 <= x_pp <= x + model.boxes[j][0] - 1
    #                                                         for y_pp in model.ypos_lst[j-1] if y - model.boxes[j][1] + 1 <= y_pp <= y + model.boxes[j][1] - 1
    #                                             ),
    #                                             model.boxes[i][0] * model.boxes[i][1] * model.a[x, y, z, i, k, t, v].X,
    #                                             "Fail Constraint 13"
    #                                         )

    def test_constraint_fourteen(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for i in model.boxID:
                    for k in model.nodes[1:]:
                        for v in model.vehicles:
                            for x in model.xpos_lst[i-1]:
                                for y in model.ypos_lst[i-1]:
                                    for z in model.zpos_lst[i-1]:
                                        self.assertGreaterEqual(model.l_p[k, v].X,
                                            (x + model.boxes[i][0]) * \
                                            sum(model.a[x, y, z, i, k, t, v].X for t in model.stages[:-1]),
                                            "Fail Constraint 14"                                                  
                                        )

    def test_constraint_fifteen(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for i in model.boxID:
                    for v in model.vehicles:
                        for k in model.nodes[1:]:
                            for l in model.nodes[1:]:
                                for x in model.xpos_lst[i-1]:
                                    for y in model.ypos_lst[i-1]:
                                        for z in model.zpos_lst[i-1]:
                                            self.assertGreaterEqual(
                                                x * sum(model.a[x, y, z, i, k, t, v].X for t in model.stages[:-1]) + \
                                                (1 - sum(model.a[x, y, z, i, k, t, v].X for t in model.stages[:-1])) * model.M1 + \
                                                (1 - sum(model.d[k, l, v, t].X for t in model.stages[:-1])) * model.M2,
                                                model.l_p[l, v].X - model.maximum_reach[i-1][k-2], # i starts at 1, k at 2 but are indexed at 0.
                                                "Fail Constraint 15"
                                            )
    
    def test_constraint_sixteen(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):        
                for k in model.nodes[1:]:
                    for l in model.nodes[1:]:
                        for v in model.vehicles:
                            self.assertGreaterEqual(
                                model.l_p[k, v].X +
                                (1 - sum(model.d[k, l, v, t].X for t in model.stages[:-1])) * model.M3,
                                model.l_p[l, v].X,
                                "Fail Constraint 16"
                            )
    
    def test_constraint_seventeen(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for k in model.nodes[1:]:
                    for v in model.vehicles:
                        self.assertGreaterEqual(
                            model.dimensions["length"],
                            model.l_p[k, v].X,
                            "Fail Constraint 17"
                        )

    def test_constraint_eighteen(self):
        for case_name, model in self.solved_models:
            with self.subTest(case=case_name):
                for x_p in model.xpos:
                    for y_p in model.ypos:
                        for z_p in model.zpos:
                            for v in model.vehicles:
                                self.assertGreaterEqual(
                                    sum(model.sigma[i-1] * model.a[x, y, z, i, k, t, v].X
                                                for i in model.boxID
                                                for k in model.nodes[1:]
                                                for t in model.stages[:-1]
                                                for x in model.xpos_lst[i-1] if x_p - model.boxes[i][0] + 1 <= x <= x_p
                                                for y in model.ypos_lst[i-1] if y_p - model.boxes[i][1] + 1 <= y <= y_p
                                                for z in model.zpos_lst[i-1] if z_p - model.boxes[i][2] + 1 <= z <= z_p
                                    ),
                                    sum((model.p[j-1] / (model.boxes[j][0] * model.boxes[j][1])) * model.a[x_pp, y_pp, z_pp, j, l, u, v].X
                                                for j in model.boxID
                                                for l in model.nodes[1:]
                                                for u in model.nodes[:-1]
                                                for x_pp in model.xpos_lst[j-1] if x_p - model.boxes[j][0] + 1 <= x_pp <= x_p
                                                for y_pp in model.ypos_lst[j-1] if y_p - model.boxes[j][1] + 1 <= y_pp <= y_p
                                                for z_pp in model.zpos_lst[j-1] if z_p + 1 <= z_pp <= model.dimensions["height"] - model.boxes[j][2]
                                    ),
                                    "Fail Constraint 18"
                                )
    
    def test_impossible_scenarios(self):
        """
        Docstring for test_impossible_scenarios
        
        The tests listed here were created to be impossible, if they turn out to be feasible, something has gone wrong
        If any of the other tests turn out infeasible, something is wrong aswell
        """

        scenario_names = [scenario["name"] for scenario in self.test_scenarios]
        impossible_tests = ["NotSoPerfectFit1", "NotSoPerfectFit2", "NotSoPerfectFit3", "NotSoPerfectFit4"]

        infeasible_names = {name for name, _ in self.infeasible_models}
        solved_names = set(name for name, _ in self.solved_models)

        for name in scenario_names:
            if name in impossible_tests:
                self.assertIn(
                    name,
                    infeasible_names,
                    msg=f"{name} should be infeasible but is not"
                )
            else:
                self.assertIn(
                    name,
                    solved_names,
                    msg=f"{name} should be feasible but is not"
                )
    
    def test_obvious_solutions(self):
        line_model = get_model(self, "StaightLine_1veh")
        self.assertAlmostEqual((line_model.a[8, 0, 0, 1, 2, 1, 0].X +
                                line_model.a[6, 0, 0, 1, 3, 2, 0].X +
                                line_model.a[4, 0, 0, 1, 4, 3, 0].X +
                                line_model.a[2, 0, 0, 1, 5, 4, 0].X) or 
                                (line_model.a[6, 0, 0, 1, 2, 1, 0].X +
                                line_model.a[4, 0, 0, 1, 3, 2, 0].X +
                                line_model.a[2, 0, 0, 1, 4, 3, 0].X +
                                line_model.a[0, 0, 0, 1, 5, 4, 0].X), 4)
        
        line_2box_model = get_model(self, "StaightLine_1veh2box")
        print(line_2box_model.a)
        self.assertAlmostEqual((line_2box_model.a[7, 0, 0, 1, 5, 1, 0].X +
                                line_2box_model.a[5, 0, 0, 1, 4, 2, 0].X +
                                line_2box_model.a[3, 0, 0, 1, 3, 3, 0].X + 
                                line_2box_model.a[2, 0, 0, 2, 3, 3, 0].X + 
                                line_2box_model.a[0, 0, 0, 1, 2, 4, 0].X) or 
                               (line_2box_model.a[7, 0, 0, 1, 5, 1, 0].X +
                                line_2box_model.a[5, 0, 0, 1, 4, 2, 0].X +
                                line_2box_model.a[1, 0, 0, 1, 3, 3, 0].X + 
                                line_2box_model.a[3, 0, 0, 2, 3, 3, 0].X + 
                                line_2box_model.a[0, 0, 0, 1, 2, 4, 0].X) , 5)          








if __name__ == "__main__":
    unittest.main()