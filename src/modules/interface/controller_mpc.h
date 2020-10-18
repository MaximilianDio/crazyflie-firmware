/*
 * controller_mpc.h
 *
 *  Created on: Oct 18, 2020
 *      Author: Maximilian Dio
 */

#ifndef __CONTROLLER_MPC_H_
#define __CONTROLLER_MPC_H_

#include "stabilizer_types.h"

void controllerMPCInit(void);
bool controllerMPCTest(void);
void controllerMPC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif /* __CONTROLLER_MPC_H_ */
