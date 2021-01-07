/*
 * position_controller_mpc.c
 *
 *  Created on: Nov 9, 2020
 *      Author: maximilian
 */

#include <math.h>

#include "vertical_controller_pid.h"
#include "num.h"
#include "commander.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "debug.h"

struct pidInit_s {
	float kp;
	float ki;
	float kd;
};

struct pidAxis_s {
	PidObject pid;

	struct pidInit_s init;
	stab_mode_t previousMode;
	float setpoint;

	float output;
};

struct this_s {
	struct pidAxis_s pidVZ;

	struct pidAxis_s pidZ;

	uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
	uint16_t thrustMin;  // Minimum thrust value to output
};

// Velocity maximums
static float zVelMax = 1.0f;
static float velMaxOverhead = 1.10f;
static const float thrustScale = 1000.0f;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true

#ifndef UNIT_TEST
static struct this_s this = {

.pidVZ = { .init = { .kp = 25, .ki = 15, .kd = 0, }, .pid.dt = DT , },

.pidZ = { .init = { .kp = 2.0f, .ki = 0.5, .kd = 0, }, .pid.dt = DT , },

.thrustBase = 36000, .thrustMin = 20000, };
#endif

// initialize controller
void verticalControllerPIDInit() {
	// Initialize PID for height control
	pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp,
			this.pidZ.init.ki, this.pidZ.init.kd, this.pidZ.pid.dt,
			POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
	pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp,
			this.pidVZ.init.ki, this.pidVZ.init.kd, this.pidVZ.pid.dt,
			POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

}

static float runPid(float input, struct pidAxis_s *axis, float setpoint,
		float dt) {
	axis->setpoint = setpoint;

	pidSetDesired(&axis->pid, axis->setpoint);
	return pidUpdate(&axis->pid, input, true);
}

// calculates desired thrust, roll, pitch to approach a 3D position setpoint
void verticalControllerPID(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state) {

	// The ROS landing detector will prematurely trip if
	// this value is below 0.5
	this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f) * velMaxOverhead;

	if (setpoint->mode.z == modeAbs) {
		setpoint->velocity.z = runPid(state->position.z, &this.pidZ,
				setpoint->position.z, DT);
	}

	verticalVelocityControllerPID(thrust, attitude, setpoint, state);

}

void verticalVelocityControllerPID(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state) {

	// Set the output limit to the maximum thrust range
	this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
	//this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

	// Thrust
	float thrustRaw = runPid(state->velocity.z, &this.pidVZ,
			setpoint->velocity.z, DT);
	// Scale the thrust and add feed forward term
	*thrust = thrustRaw * thrustScale + this.thrustBase;
	// Check for minimum thrust
	if (*thrust < this.thrustMin) {
		*thrust = this.thrustMin;
	}
}

// resets control input
void verticalControllerPIDResetAll() {

	pidReset(&this.pidZ.pid);

	pidReset(&this.pidVZ.pid);
}
