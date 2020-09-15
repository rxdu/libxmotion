/* 
 * sensor_if.hpp
 * 
 * Created on: Nov 24, 2018 05:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef SENSOR_IF_HPP
#define SENSOR_IF_HPP

namespace librav
{
template<typename CtrlParamType, typename StateType, typename InputType, typename OutputType>
class SensorInterface
{
public:
	ControllerInterface(const StateType& state):
		state_(state),initialized_(false){};
	virtual ~ControllerInterface() = default;

	typedef CtrlParamType ParamType;

	virtual void InitParams(const CtrlParamType& param) = 0;
	virtual void Update(const InputType& desired, OutputType* cmd) = 0;

protected:
	const StateType& state_;
	CtrlParamType param_;
	bool initialized_;
};
}
#endif /* SENSOR_IF_HPP */
