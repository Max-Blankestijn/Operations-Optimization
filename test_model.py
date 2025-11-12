import unittest
import numpy as np

from model import CVRP

class TestCVRP(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """
        Create multiple test setups to be reused across all constraint tests.
        Each setup is a dict, all tests should work for every possible setup.
        This is NOT for making deterministic tests we can hand verify, I'll do that in a seperate file I think
        """

        cls.test_inputs = []

        # Max: Lets add helper functions to a separate file helper.py and import them
        def make_links(nodes):
            """Helper: make symmetric distance dict for all node pairs."""
            
            np.random.seed(42)

            links = {}
            for i in nodes:
                for j in nodes:
                    if i == j:
                        links[(i, j)] = {"distance": 9999}
                    else:
                        dist = np.random.randint(5, 100)
                        links[(i, j)] = {"distance": dist}
                        links[(j, i)] = {"distance": dist}
            return links

        # Different configurations, this list will become quite big I think
        test_scenarios = [
            {"name": "Small_1veh_3nodes", "nodes": [1, 2, 3], "vehicles": [0]},
            {"name": "Small_2veh_3nodes", "nodes": [1, 2, 3], "vehicles": [0, 1]},
            {"name": "Medium_1veh_6nodes", "nodes": list(range(1, 7)), "vehicles": [0]},
            {"name": "Medium_2veh_6nodes", "nodes": list(range(1, 7)), "vehicles": [0, 1]},
            {"name": "Large_3veh_20nodes", "nodes": list(range(1, 21)), "vehicles": [0, 1, 2]},
        ]

        for scenario in test_scenarios:
            nodes = scenario["nodes"]
            links = make_links(nodes)

            # Will be added in the dict of test_scenarios once they actually do stuff
            dimensions = {"length": 10, "width": 5, "height": 5}
            boxes = {"box1": [1, 1, 1]}

            cls.test_inputs.append({
                "name": scenario["name"],
                "nodes": nodes,
                "links": links,
                "vehicles": scenario["vehicles"],
                "dimensions": dimensions,
                "boxes": boxes
            })


    def build_model(self, case, constraints):
        """Build and return a CVRP model for a given test case."""
        return CVRP(
            case["name"],
            case["nodes"],
            case["links"],
            case["vehicles"],
            case["dimensions"],
            case["boxes"],
            constraints=constraints
        )

    def test_constraint_two(self):
        for case in self.test_inputs:
            with self.subTest(case=case["name"]):
                # Max: I recommend adding a function that takes as input a range and returns the constraint dictionary
                # associated with the active constraints. (Take full constraint dict and set disabled to false)
                testmodel = self.build_model(case, {"constraintTwo": True})
                #Still need to write the actual test lol but je suis tired





if __name__ == "__main__":
    unittest.main()