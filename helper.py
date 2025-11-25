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
                   "constraintElevenDummy": None}

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

if __name__ == "__main__":
    # Example usage of constraintGenerator with both range and list
    constraint_dict = constraintGenerator(range(1, 9))
    print(constraint_dict)
    constraint_dict = constraintGenerator([2, 3, 5, 8])
    print(constraint_dict)