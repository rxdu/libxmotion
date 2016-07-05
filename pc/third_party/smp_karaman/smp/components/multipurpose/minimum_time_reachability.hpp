#ifndef _SMP_MINIMUM_TIME_REACHABILITY_HPP_
#define _SMP_MINIMUM_TIME_REACHABILITY_HPP_

#include <smp/components/multipurpose/minimum_time_reachability.h>

#include <smp/planners/rrtstar.hpp>
#include <smp/common/region.hpp>
#include <smp/components/model_checkers/base.hpp>
#include <smp/components/cost_evaluators/base.hpp>



template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::minimum_time_reachability () {

  min_cost_vertex = NULL;

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 0.0;
  }
    
}


template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::~minimum_time_reachability () {

  
}


template < class typeparams, int NUM_DIMENSIONS >
smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::minimum_time_reachability (const region_t &region_in) {
  
  region_goal = region_in;
}


template < class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::set_goal_region (const region_t &region_in) {

  region_goal = region_in;
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::ce_update_vertex_cost (vertex_t *vertex_in) {
    
  if (vertex_in->data.reaches_goal == true) {
    
    bool update_trajectory = false;
    
    if (min_cost_vertex == NULL) {
      min_cost_vertex = vertex_in;
      cout << "COST -- : " << vertex_in->data.total_cost << endl;
      update_trajectory = true;
    }
    
    
    if ( (vertex_in->data.total_cost < min_cost_vertex->data.total_cost) ) {
      cout << "COST -- : " << vertex_in->data.total_cost << endl;
      min_cost_vertex = vertex_in;
      update_trajectory = true;
    }

    
    if (update_trajectory == true) {

      min_cost_trajectory.clear_delete ();

      vertex_t *vertex_ptr = min_cost_vertex;
      while (1) {

	edge_t *edge_curr = vertex_ptr->incoming_edges.back();

	trajectory_t *trajectory_curr = edge_curr->trajectory_edge;
	min_cost_trajectory.list_states.push_front (new state_t(*(vertex_ptr->state)));

	if (vertex_ptr->incoming_edges.size() == 0)
	  break;
    
	for (typename list<state_t*>::iterator it_state = trajectory_curr->list_states.begin();
	     it_state != trajectory_curr->list_states.end(); it_state++) {
	  min_cost_trajectory.list_states.push_front (new state_t(**it_state));
	}

	for (typename list<input_t*>::iterator it_input = trajectory_curr->list_inputs.begin();
	     it_input != trajectory_curr->list_inputs.end(); it_input++) {
	  min_cost_trajectory.list_inputs.push_front (new input_t(**it_input));
	}

	vertex_ptr = edge_curr->vertex_src;
      }  

      // Call all the update functions 
      for (typename list<update_func_t>::iterator it_func = list_update_functions.begin(); 
	   it_func != list_update_functions.end(); it_func++) {
	
	(*it_func)(&min_cost_trajectory);
      }


    }
  }


  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::ce_update_edge_cost (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::mc_update_insert_vertex (vertex_t *vertex_in) {

  for (int i = 0; i < NUM_DIMENSIONS; i++)  {
    if (fabs (vertex_in->state->state_vars[i] - region_goal.center[i]) > region_goal.size[i]) {
      vertex_in->data.reaches_goal = false;
      return 1;
    }
  }
  
  vertex_in->data.reaches_goal = true;
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::mc_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::mc_update_delete_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::mc_update_delete_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::get_solution (trajectory_t &trajectory_out) {


  if (!min_cost_vertex)
    return 1;

  trajectory_out.clear();

  for (typename list<state_t*>::iterator it_state = min_cost_trajectory.list_states.begin();
       it_state != min_cost_trajectory.list_states.end(); it_state++) {
    
    trajectory_out.list_states.push_front (new state_t(**it_state));
  }

  for (typename list<input_t*>::iterator it_input = min_cost_trajectory.list_inputs.begin();
       it_input != min_cost_trajectory.list_inputs.end(); it_input++) {
    trajectory_out.list_inputs.push_front (new input_t(**it_input));
  }



  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::evaluate_cost_trajectory (state_t *state_initial_in,
			    trajectory_t *trajectory_in,
			    state_t *state_final_in) {
  
  double total_time = 0.0;
  for (typename list<input_t*>::iterator iter = trajectory_in->list_inputs.begin();
       iter != trajectory_in->list_inputs.end(); iter++) {
    
    input_t *input_curr = *iter;
    total_time += (*input_curr)[0];
    
  }
  
  return total_time;

}


template< class typeparams, int NUM_DIMENSIONS >
double smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::get_best_cost () {
  
  if (min_cost_vertex == NULL)
    return -1.0;
  else
    return (double)(min_cost_vertex->data.total_cost);
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::clear_update_function_list () {
  
  list_update_functions.clear();

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::minimum_time_reachability<typeparams,NUM_DIMENSIONS> 
::register_new_update_function (update_func_t update_function) {

  if (update_function == NULL) 
    return 0;

  list_update_functions.push_back (update_function);
  
  return 1;
}


#endif
