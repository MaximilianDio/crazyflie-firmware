/*
 * horizontal_controller_mpc.h
 *
 *  Created on: Jan 4, 2021
 *      Author: maximilian
 */

#ifndef SRC_MODULES_INTERFACE_HORIZONTAL_CONTROLLER_MPC_H_
#define SRC_MODULES_INTERFACE_HORIZONTAL_CONTROLLER_MPC_H_

#include "stabilizer_types.h"

// calculates optimal thrust, roll, pitch based on model predictive control approach to reach 3D position setpoint

// initialize controller
void horizontalControllerMPCInit();

// resets control input
void horizontalControllerMPCResetAll();

// calculates desired thrust, roll, pitch to approach a 3D position setpoint
void horizontalControllerMPC(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state);

void horizontalVelocityControllerMPC(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state);


#endif /* SRC_MODULES_INTERFACE_HORIZONTAL_CONTROLLER_MPC_H_ */
