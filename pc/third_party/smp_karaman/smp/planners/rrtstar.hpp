#ifndef _SMP_RRTSTAR_HPP_
#define _SMP_RRTSTAR_HPP_


#include <smp/planners/rrtstar.h>


#include <smp/planners/base_incremental.hpp>
#include <smp/planners/planner_parameters.hpp>

#include <smp/components/cost_evaluators/base.hpp>



template< class typeparams >
smp::rrtstar<typeparams>
::rrtstar () {
  
  cost_evaluator = NULL;
}


template< class typeparams >
smp::rrtstar<typeparams>
::~rrtstar () {
  
}



template< class typeparams >
smp::rrtstar<typeparams>
::rrtstar (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
	   collision_checker_t &collision_checker_in, model_checker_t &model_checker_in, cost_evaluator_t &cost_evaluator_in) :
  planner_incremental<typeparams>(sampler_in, distance_evaluator_in, extender_in, collision_checker_in, model_checker_in),
  cost_evaluator(cost_evaluator_in) {
  
  
  
}


template< class typeparams >
int smp::rrtstar<typeparams>
::initialize (state_t *initial_state_in) {

  planner_incremental_t::initialize(initial_state_in);
  
  this->root_vertex->data.total_cost = 0;
  
  return 1;
}

template< class typeparams >
int smp::rrtstar<typeparams>
::init_cost_evaluator (cost_evaluator_t &cost_evaluator_in) {
  
  cost_evaluator = cost_evaluator_in;

  return 1;
}


template< class typeparams >
int smp::rrtstar<typeparams>
::propagate_cost (vertex_t *vertex_in, double total_cost_new) {
  
  // Update the cost of this vertex
  vertex_in->data.total_cost = total_cost_new;
  
  cost_evaluator.ce_update_vertex_cost (vertex_in);
  
  // Recursively propagate the cost along the edges
  for (typename list<edge_t*>::iterator iter_edge = vertex_in->outgoing_edges.begin(); 
       iter_edge != vertex_in->outgoing_edges.end(); iter_edge++) {


    edge_t *edge_curr = *iter_edge;
    
    vertex_t *vertex_next = edge_curr->vertex_dst;

    if (vertex_next != vertex_in)
      this->propagate_cost (vertex_next, vertex_in->data.total_cost + edge_curr->data.edge_cost);
  }
  
  return 1;
}


template< class typeparams >
int smp::rrtstar<typeparams>
::iteration () {
  
  // TODO: Check whether the rrtstar is initialized properly (including its base classes)
  
  // 1. Sample a new state from the obstacle-free space
  state_t *state_sample;
  this->sampler.sample (&state_sample);
  if (this->collision_checker.check_collision_state (state_sample) == 0) {
    delete state_sample;
    return 0; 
  }
  
  
  // 2. Find the nearest vertex
  vertex_t *vertex_nearest;
  this->distance_evaluator.find_nearest_vertex (state_sample, (void **)&vertex_nearest);

  
  // 3. Extend the nearest vertex towards the sample
  
  double radius;
  if (parameters.get_fixed_radius() < 0.0) {
    double num_vertices = (double)(this->get_num_vertices());
    radius = parameters.get_gamma() * pow (log(num_vertices)/num_vertices,  1.0 /( (double)(parameters.get_dimension()) )  );

    // if (this->get_num_vertices()%1000 == 0) 
    //   cout << "radius " << radius << endl; 

    if (radius > parameters.get_max_radius())
      radius = parameters.get_max_radius();
  }
  else 
    radius = parameters.get_fixed_radius();

  radius_last = radius;

  int exact_connection = -1;
  trajectory_t *trajectory = new trajectory_t;
  list<state_t*> *intermediate_vertices = new list<state_t*>;
  if (this->extender.extend (vertex_nearest->state, state_sample,
			     &exact_connection, trajectory, intermediate_vertices) == 1) {  // If the extension is successful
    
    
    // 4. Check the new trajectory for collision
    if (check_extended_trajectory_for_collision (vertex_nearest->state, trajectory) == 1) {  // If the trajectory is collision free
      
      // 5. Find the parent state
      vertex_t *vertex_parent = vertex_nearest;
      trajectory_t *trajectory_parent = trajectory;
      list<state_t*> *intermediate_vertices_parent = intermediate_vertices;
      
      double cost_trajectory_from_parent = this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_parent);
      double cost_parent = vertex_parent->data.total_cost + cost_trajectory_from_parent;

      
      // Define the new variables that are used in both phase 1 and 2.
      list<void*> list_vertices_in_ball;
      state_t *state_extended = NULL;
      
      if (parameters.get_phase() >= 1) {  // Check whether phase 1 should occur.
      
	state_extended = new state_t(*(trajectory_parent->list_states.back())); // Create a copy of the final state
      
	// Compute the set of all nodes that reside in a ball of a certain radius centered at the extended state
	this->distance_evaluator.find_near_vertices_r (state_extended, radius, &list_vertices_in_ball);

	for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
	  vertex_t *vertex_curr = (vertex_t*)(*iter);
	
	  // Skip if current vertex is the same as the nearest vertex
	  if (vertex_curr == vertex_nearest) 
	    continue;
	
	  // Attempt an extension from vertex_curr to the extended state
	  trajectory_t  *trajectory_curr = new trajectory_t;
	  list<state_t*> *intermediate_vertices_curr = new list<state_t*>;
	  exact_connection = -1;
	  if (this->extender.extend (vertex_curr->state, state_extended, 
				     &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1) {

	    if ( (exact_connection == 1) && (check_extended_trajectory_for_collision(vertex_curr->state,trajectory_curr) == 1) ) {

	      // Calculate the cost to get to the extended state with the new trajectory
	      double cost_trajectory_from_curr = 
		this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_curr);
	      double cost_curr = vertex_curr->data.total_cost + cost_trajectory_from_curr;
	    
	      // Check whether the total cost through the new vertex is less than the parent
	      if (cost_curr < cost_parent) {
	      
		// Make new vertex the parent vertex
		vertex_parent = vertex_curr;

		trajectory_t *trajectory_tmp = trajectory_parent; // Swap trajectory_parent and trajectory_curr
		trajectory_parent = trajectory_curr;              //   to properly free the memory later
		trajectory_curr = trajectory_tmp;
	      
		list<state_t*> *intermediate_vertices_tmp = intermediate_vertices_parent;  // Swap the intermediate vertices
		intermediate_vertices_parent = intermediate_vertices_curr;                   //   to properly free the memory later
		intermediate_vertices_curr = intermediate_vertices_tmp;

		cost_trajectory_from_parent = cost_trajectory_from_curr;
		cost_parent = cost_curr;
	      }
	    }
	  }
	
	  delete trajectory_curr;
	  delete intermediate_vertices_curr;
	}
      }
      
      // Create a new vertex
      this->insert_trajectory (vertex_parent, trajectory_parent, intermediate_vertices_parent);
      
      // Update the cost of the edge and the vertex      
      vertex_t *vertex_last = this->list_vertices.back();
      vertex_last->data.total_cost = cost_parent;
      cost_evaluator.ce_update_vertex_cost (vertex_last);
      
      edge_t *edge_last = vertex_parent->outgoing_edges.back();
      edge_last->data.edge_cost = cost_trajectory_from_parent;


      if (parameters.get_phase() >= 2) {  // Check whether phase 2 should occur
      

	// 6. Extend from the new vertex to the existing vertices in the ball to rewire the tree
	for (list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
	
	  vertex_t *vertex_curr = (vertex_t*)(*iter);

	  if (vertex_curr == vertex_last)
	    continue;
	
	  // Attempt an extension from the extended vertex to the current vertex
	  trajectory_t *trajectory_curr = new trajectory_t;
	  list<state_t*> *intermediate_vertices_curr = new list<state_t*>;
	  bool free_tmp_memory = true;
	  exact_connection = -1;
	  if (this->extender.extend (vertex_last->state, vertex_curr->state,
				     &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1) {
	  
	    if ( (exact_connection == 1) && (check_extended_trajectory_for_collision(vertex_last->state,trajectory_curr) == 1) ) {

	      // Calculate the cost to get to the extended state with the new trajectory
	      double cost_trajectory_to_curr = 
		this->cost_evaluator.evaluate_cost_trajectory (vertex_last->state, trajectory_curr);
	      double cost_curr = vertex_last->data.total_cost + cost_trajectory_to_curr;
	    
	    
	      // Check whether cost of the trajectory through vertex_last is less than the current trajectory
	      if (cost_curr < vertex_curr->data.total_cost) {
	      
	      
		// Delete the old parent of vertex_curr
		edge_t *edge_parent_curr = vertex_curr->incoming_edges.back();
		this->delete_edge (edge_parent_curr);
	      
		// Add vertex_curr's new parent
		this->insert_trajectory (vertex_last, trajectory_curr, intermediate_vertices_curr, vertex_curr);
		edge_t *edge_curr = vertex_curr->incoming_edges.back();
		edge_curr->data.edge_cost = cost_trajectory_to_curr;

		free_tmp_memory = false;
	      
		// Propagate the cost
		this->propagate_cost (vertex_curr, vertex_last->data.total_cost + edge_curr->data.edge_cost);	      

	      }
	    }
	  }

	  if (free_tmp_memory == true) {
	    delete trajectory_curr;
	    delete intermediate_vertices_curr;
	  }
	
	}
      }

      // Completed all phases, return with success
      delete state_sample;
      if (state_extended)
	delete state_extended;
      return 1;
    }
  }
  

  // 7. Handle the error case
  // If the first extension was not successful, or the trajectory was not collision free,
  //     then free the memory and return failure
  delete state_sample;
  delete trajectory;
  delete intermediate_vertices;
  
  return 0;

}


#endif
