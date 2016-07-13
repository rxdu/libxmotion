#ifndef _SMP_SYSTEM_DUBINS_HPP_
#define _SMP_SYSTEM_DUBINS_HPP_

#define DISTANCE_LIMIT 100.0
#define DELTA_DISTANCE 0.25
#define TURNING_RADIUS 1.00


#ifndef DBL_MAX
#define DBL_MAX 10000000000000000.0
#endif

#include <quadrotor.h>

#include <smp/components/extenders/state_array_double.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/components/extenders/base.hpp>

#include <cmath>
#include <cstdlib>

template< class typeparams >
int smp::extender_quadrotor<typeparams>
::ex_update_insert_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams >
int smp::extender_quadrotor<typeparams>
::ex_update_insert_edge (edge_t *edge_in) { 

  return 1;
}


template< class typeparams >
int smp::extender_quadrotor<typeparams>
::ex_update_delete_vertex (vertex_t *vertex_in){
  
  return 1;
}


template< class typeparams >
int smp::extender_quadrotor<typeparams>
::ex_update_delete_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams >
smp::extender_quadrotor<typeparams>
::extender_quadrotor () {

}


template< class typeparams >
smp::extender_quadrotor<typeparams>
::~extender_quadrotor () {

}


template< class typeparams >
int smp::extender_quadrotor<typeparams>
::extend (state_t *state_from_in, state_t *state_towards_in,
	  int *exact_connection_out, trajectory_t *trajectory_out,
	  list<state_t*> *intermediate_vertices_out) {
  
  
//  if (extend_dubins_all (state_from_in, state_towards_in, exact_connection_out,
//			 &(trajectory_out->list_states), &(trajectory_out->list_inputs)) < 0.0) {
//
//    return 0;
//  }
  
  

  return 1;
}
 

#endif
