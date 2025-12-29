import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def constraintGenerator(active) -> dict:
    '''
    Determines constraints that will be active in the model, input can be either a list or a range of active constraints
    Keep in mind that the inserted range should go up to constraint n + 1 rather than n.
    '''
    # Dictionary of all constraints, constraint one, six and eleven are dummy constraints that are always active
    # and used for incremental counting
    constraints = {"constraintOneDummy": None,
                   "constraintTwo": False,
                   "constraintThree": False,
                   "constraintFour": False,
                   "constraintFive": False,
                   "constraintSixDummy": None,
                   "constraintSeven": False,
                   "constraintEight": False,
                   "constraintNine": False,
                   "constraintTen": False,
                   "constraintEleven": False}

    # Range Activation (range(constraint_start, constraint_end+1))
    if isinstance(active, range):
        enum = 1
        for key, value in constraints.items():
            if value != None and enum in active:
                constraints[key] = True
            enum += 1

    # List Activation (List values should be the constraint numbers)
    elif isinstance(active, list):
        enum = 1
        for key, value in constraints.items():
            if value != None and enum in active:
                constraints[key] = True
            enum += 1

    # Error handling
    else:
        raise TypeError("Invalid Type Provided to constraintGenerator. Provide a range or list")

    return constraints

def plot_boxes_3d(used_boxes, boxes, dimensions):
    """
    visualizes boxes in 3D spaces
    """

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Assign distinct colors for box types
    colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'orange', 'yellow']

    for idx, (boxID, placements) in enumerate(used_boxes.items()):
        color = colors[idx % len(colors)]
        L, W, H = boxes[boxID]

        for (x, y, z) in placements:
            # List of vertices for the rectangular prism
            vertices = [
                [x, y, z],
                [x + L, y, z],
                [x + L, y + W, z],
                [x, y + W, z],
                [x, y, z + H],
                [x + L, y, z + H],
                [x + L, y + W, z + H],
                [x, y + W, z + H]
            ]

            # Define the 6 faces
            faces = [
                [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
                [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
                [vertices[0], vertices[1], vertices[5], vertices[4]],  # side
                [vertices[2], vertices[3], vertices[7], vertices[6]],  # side
                [vertices[1], vertices[2], vertices[6], vertices[5]],  # front
                [vertices[4], vertices[7], vertices[3], vertices[0]]  # back
            ]

            box = Poly3DCollection(faces, alpha=0.35)
            box.set_facecolor(color)
            box.set_edgecolor("black")
            ax.add_collection3d(box)

    ax.set_xlabel("Length")
    ax.set_ylabel("Width")
    ax.set_zlabel("Height")

    # Auto scale to fit boxes
    ax.set_xlim(0, max(dimensions["length"], dimensions["length"]))
    ax.set_ylim(0, max(dimensions["width"], dimensions["width"]))
    ax.set_zlim(0, max(dimensions["height"], dimensions["height"]))

    ax.set_title("3D Loaded Vehicle Visualization")
    plt.tight_layout()
    plt.show()

def make_links(nodes):
    # Generate links from each node to each other node with random distances, might need to change to account for depot
    links = {(i, j): {"distance": np.random.randint(10, 50) if i != j else 9999999} for i in nodes for j in nodes}

    # Make it symmetric
    for i, j in list(links.keys()):
        if i != j:
            links[(j, i)] = {"distance": links[(i, j)]["distance"]}
    return links


if __name__ == "__main__":
    # Example usage of constraintGenerator with both range and list
    constraint_dict = constraintGenerator(range(1, 9))
    print(constraint_dict)
    constraint_dict = constraintGenerator([2, 3, 5, 8])
    print(constraint_dict)