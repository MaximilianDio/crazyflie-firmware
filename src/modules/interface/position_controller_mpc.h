/*
 * position_controller_mpc.h
 *
 *  Created on: Nov 9, 2020
 *      Author: maximilian
 */

#ifndef SRC_MODULES_INTERFACE_POSITION_CONTROLLER_MPC_H_
#define SRC_MODULES_INTERFACE_POSITION_CONTROLLER_MPC_H_

#include "stabilizer_types.h"

// calculates optimal thrust, roll, pitch based on model predictive control approach to reach 3D position setpoint

// initialize controller
void positionControllerMPCInit();

// resets control input
void positionControllerMPCResetAll();

// calculates desired thrust, roll, pitch to approach a 3D position setpoint
void positionControllerMPC(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state);

void velocityControllerMPC(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state);


#endif /* SRC_MODULES_INTERFACE_POSITION_CONTROLLER_MPC_H_ */
