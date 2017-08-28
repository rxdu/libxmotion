/*
 * sim_env_gen.h
 *
 *  Created on: Aug 28, 2017
 *      Author: rdu
 */

#ifndef QUADROTOR_PATH_REPAIR_SIM_ENV_GEN_H_
#define QUADROTOR_PATH_REPAIR_SIM_ENV_GEN_H_

namespace librav
{

class SimEnvGen
{
public:
	SimEnvGen() = default;
	~SimEnvGen() = default;

private:
	void GenerateSpace(double x, double y, double z); 
};

}

#endif /* QUADROTOR_PATH_REPAIR_SIM_ENV_GEN_H_ */
