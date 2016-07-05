// Standard header files
#include<iostream>
using namespace std;


// SMP HEADER FILES ------
#include <smp/components/samplers/uniform.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/dubins.hpp>
#include <smp/components/collision_checkers/standard.hpp>
#include <smp/components/multipurpose/minimum_time_reachability.hpp>

#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>


// SMP TYPE DEFINITIONS -------
using namespace smp;

// State, input, vertex_data, and edge_data definitions
typedef state_dubins state_t;
typedef input_dubins input_t;
typedef minimum_time_reachability_vertex_data vertex_data_t;
typedef minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef state_t state;
  typedef input_t input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
} typeparams;

// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef sampler_uniform<typeparams,3> sampler_t;
typedef distance_evaluator_kdtree<typeparams,3> distance_evaluator_t;
typedef extender_dubins<typeparams> extender_t;
typedef collision_checker_standard<typeparams,2> collision_checker_t;
typedef minimum_time_reachability<typeparams,2> min_time_reachability_t;

// Define all algorithm types
typedef rrtstar<typeparams>  rrtstar_t;






int
main () {




  // 1. CREATE PLANNING OBJECTS

  // 1.a Create the components
  sampler_t sampler;
  distance_evaluator_t distance_evaluator;
  extender_t extender;
  collision_checker_t collision_checker;
  min_time_reachability_t min_time_reachability;

  // 1.b Create the planner algorithm -- Note that the min_time_reachability variable acts both
  //                                       as a model checker and a cost evaluator.
  rrtstar_t planner (sampler, distance_evaluator, extender, collision_checker,
		     min_time_reachability, min_time_reachability);

  planner.parameters.set_phase (2);   // The phase parameter can be used to run the algorithm as an RRT, 
                                      // See the documentation of the RRG algorithm for more information.

  planner.parameters.set_gamma (35.0);    // Set this parameter should be set at least to the side length of
                                          //   the (bounded) state space. E.g., if the state space is a box
                                          //   with side length L, then this parameter should be set to at
                                          //   least L for rapid and efficient convergence in trajectory space.
  planner.parameters.set_dimension (3);
  planner.parameters.set_max_radius (20.0);  // This parameter should be set to a high enough value. In practice,
                                             //   one can use smaller values of this parameter to get a good
                                             //   solution quickly, while preserving the asymptotic optimality.






  // 2. INITALIZE PLANNING OBJECTS

  // 2. Initialize the sampler component
  region<3> sampler_support;
  sampler_support.center[0] = 0.0;
  sampler_support.center[1] = 0.0;
  sampler_support.center[2] = 0.0;
  sampler_support.size[0] = 20.0;
  sampler_support.size[1] = 20.0;
  sampler_support.size[2] = 2.0*M_PI;
  sampler.set_support (sampler_support);


  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.


  // 2.c Initialize the extender


  // 2.d Initialize the collision checker
  region<2> obstacle_new;
  for (int i = 0; i < 2; i++) {
    obstacle_new.center[i] = 5.0;
    obstacle_new.size[i] = 5.0;
  }
  collision_checker.add_obstacle (obstacle_new);


  // 2.e Initialize the model checker and the cost evaluator
  region<2> region_goal;
  region_goal.center[0] = 8.0;
  region_goal.center[1] = 8.0;
  region_goal.size[0] = 2.0;
  region_goal.size[1] = 2.0;
  min_time_reachability.set_goal_region (region_goal);


  // 2.f Initialize the planner
  state_t *state_initial = new state_t;
  for (int i = 0; i < 3; i++) {
    state_initial->state_vars[i] = 0.0;
  }
  planner.initialize (state_initial);







  // 3. RUN THE PLANNER
  for (int i = 0; i < 2000; i++){
    planner.iteration ();

    if (i%100 == 0) {
      cout << "Iteration: " << i << endl;
    }
  }







  // 4. GET THE RESULTS
  trajectory_t trajectory_final;
  min_time_reachability.get_solution (trajectory_final);




  return 1;

}
