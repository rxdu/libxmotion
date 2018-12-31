# Library of Sampling-Based Planning Algorithms

## Steps to perform search with a planner

1. Define the planning space
2. Specify start and goal state (region)
3. Specify Steer function and StateValidityCheck/PathValidityCheck function
4. Perfrom search

## Steps to construct a new planner

1. Define a planning space derived from SpaceBase
   * define StateType (derived from State)
   * define EvaluateDistance()
   * define sampling methods and other helper functions
2. Use the TreeAdapter to wrap a tree data structure for RRT-variant algorithms if existing ones don't work
3. Define the new planner and follow the interface defined in PlannerBase
   
