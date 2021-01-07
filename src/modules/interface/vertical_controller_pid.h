/*
 * position_controller_mpc.h
 *
 *  Created on: Nov 9, 2020
 *      Author: maximilian
 */

#ifndef SRC_MODULES_INTERFACE_VERTICAL_CONTROLLER_PID_H_
#define SRC_MODULES_INTERFACE_VERTICAL_CONTROLLER_PID_H_

#include "stabilizer_types.h"

// calculates optimal thrust, roll, pitch based on model predictive control approach to reach 3D position setpoint

// initialize controller
void verticalControllerPIDInit();

// resets control input
void verticalControllerPIDResetAll();

// calculates desired thrust, roll, pitch to approach a 3D position setpoint
void verticalControllerPID(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state);

void verticalVelocityControllerPID(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state);


#endif /* SRC_MODULES_INTERFACE_VERTICAL_CONTROLLER_PID_H_ */
