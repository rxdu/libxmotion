# Library of Sampling-Based Planning Algorithms

## Steps to perform search with a planner

1. Define the planning space
2. Specify goal region radius (normally should equal to max steering distance)
3. Specify "Steer" function and "StateValidityCheck"/"PathValidityCheck" function
4. Specify start and goal state and perfrom search

## Steps to construct a new planner

1. Define a planning space derived from SpaceBase
   * define StateType (derived from State)
   * define EvaluateDistance()
   * define sampling methods and other helper functions
2. Use the TreeAdapter to wrap a tree data structure for RRT-variant algorithms if existing ones don't work
3. Define the new planner and follow the interface defined in PlannerBase
   