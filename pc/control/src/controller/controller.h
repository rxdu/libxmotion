/*
 * controller.h
 *
 *  Created on: Jun 19, 2017
 *      Author: rdu
 */

#ifndef CONTROL_SRC_CONTROLLER_CONTROLLER_H_
#define CONTROL_SRC_CONTROLLER_CONTROLLER_H_

namespace srcl_ctrl
{

template<typename CtrlParamType, typename StateType, typename InputType, typename OutputType>
class ControllerInterface
{
public:
	ControllerInterface(const StateType& state):
		state_(state),initialized_(false){};
	virtual ~ControllerInterface() = default;

	typedef CtrlParamType ParamType;

protected:
	const StateType& state_;
	CtrlParamType param_;
	bool initialized_;

public:
	virtual void InitParams(const CtrlParamType& param) = 0;
	virtual void Update(const InputType& desired, OutputType& cmd) = 0;

};

}

#endif /* CONTROL_SRC_CONTROLLER_CONTROLLER_H_ */
