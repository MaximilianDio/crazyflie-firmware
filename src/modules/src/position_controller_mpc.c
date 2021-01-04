/*
 * position_controller_mpc.c
 *
 *  Created on: Nov 9, 2020
 *      Author: maximilian
 */

#include "position_controller_mpc.h"

#include <math.h>
#include "num.h"
#include "commander.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "debug.h"

// define which solver to use!
#define MUAO_MPC // MUAO_MPC or OSQP_MPC

#ifdef MUAO_MPC
// muAO-MPC
#include "mpc.h"

#endif

#ifdef OSQP_MPC
// OSQP
#include "workspace.h"
#include "osqp.h"
#endif

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

// Maximum roll/pitch angle permited
static float rpLimit = 20;
//static float rpLimitOverhead = 1.10f;
// Velocity maximums
//static float xyVelMax = 1.0f;
static float zVelMax = 1.0f;
static float velMaxOverhead = 1.10f;
static const float thrustScale = 1000.0f;

// for logging
static float xVelRef = 0.0;
static float yVelRef = 0.0;
static float xVel = 0.0;
static float yVel = 0.0;
static float zPos = 0.0;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true

#ifndef UNIT_TEST
static struct this_s this = {

.pidVZ = { .init = { .kp = 25, .ki = 15, .kd = 0, }, .pid.dt = DT , },

.pidZ = { .init = { .kp = 2.0f, .ki = 0.5, .kd = 0, }, .pid.dt = DT , },

.thrustBase = 36000, .thrustMin = 20000, };
#endif

// for testing purposes
static double u1;
static double u2;

#ifdef MUAO_MPC
static real_t x_ref[MPC_HOR_STATES] = { 0 };

// define reference trajectory
void createXRef(const setpoint_t *setpoint, const state_t *state) {
	// TODO

	float speedCoefficient = 0.5;
//	// transform to body frame
	float cosyaw = cosf(state->attitude.yaw * (float) M_PI / 180.0f);
	float sinyaw = sinf(state->attitude.yaw * (float) M_PI / 180.0f);
	float bodyvx = speedCoefficient * setpoint->velocity.x;
	float bodyvy = speedCoefficient * setpoint->velocity.y;

	// transform reference velocity to body frame
	xVelRef = (bodyvx * cosyaw - bodyvy * sinyaw);
	yVelRef = (bodyvy * cosyaw + bodyvx * sinyaw);

	// initialize first command
	x_ref[0] = (real_t) 0.0; // pos_x
	x_ref[1] = (real_t) 0.0; // pos_y
	x_ref[2] = (real_t) xVelRef; // vel_x
	x_ref[3] = (real_t) yVelRef; // vel_y

//	for (int i = 0; i < 3; i++) {
//		consolePrintf("x[%i] = %f \n", i, (double ) x_ref[i]);
//	}

	double decreaseValue = 1.0;

	for (int i = 1; i < MPC_HOR; i++) {
		// velocity
		// dampened by decrease_value
		x_ref[i * MPC_STATES + 2] = (real_t) x_ref[(i - 1) * MPC_STATES + 2]
				* decreaseValue; // vel_x
		x_ref[i * MPC_STATES + 3] = (real_t) x_ref[(i - 1) * MPC_STATES + 3]
				* decreaseValue; // vel_y

		// integrate over the velocity
		// pos(new) = pos(old) + dt/2*(vel(old)+vel(new))
		x_ref[i * MPC_STATES + 0] = x_ref[(i - 1) * MPC_STATES + 0]
				+ ((real_t) DT ) / 2
						* (x_ref[(i - 1) * MPC_STATES + 2]
								+ x_ref[i * MPC_STATES + 2]); // pos_x
		x_ref[i * MPC_STATES + 1] = x_ref[(i - 1) * MPC_STATES + 1]
				+ ((real_t) DT ) / 2
						* (x_ref[(i - 1) * MPC_STATES + 3]
								+ x_ref[i * MPC_STATES + 3]); // pos_y
	}

}
#endif

// initialize controller
void positionControllerMPCInit() {
	// Initialize PID for height control
	pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp,
			this.pidZ.init.ki, this.pidZ.init.kd, this.pidZ.pid.dt,
			POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
	pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp,
			this.pidVZ.init.ki, this.pidVZ.init.kd, this.pidVZ.pid.dt,
			POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

	u1 = 0.0;
	u2 = 0.0;

}

static float runPid(float input, struct pidAxis_s *axis, float setpoint,
		float dt) {
	axis->setpoint = setpoint;

	pidSetDesired(&axis->pid, axis->setpoint);
	return pidUpdate(&axis->pid, input, true);
}

// calculates desired thrust, roll, pitch to approach a 3D position setpoint
void positionControllerMPC(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state) {

	// The ROS landing detector will prematurely trip if
	// this value is below 0.5
	this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f) * velMaxOverhead;

	if (setpoint->mode.z == modeAbs) {
		setpoint->velocity.z = runPid(state->position.z, &this.pidZ,
				setpoint->position.z, DT);
	}

	velocityControllerMPC(thrust, attitude, setpoint, state);

}

void velocityControllerMPC(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state) {

	// Set the output limit to the maximum thrust range
	this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
	//this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

#ifdef MUAO_MPC
	// muAO-MPC

	real_t x[MPC_STATES]; /* current state of the system */
	extern struct mpc_ctl ctl; /* already defined */
	ctl.conf->in_iter = 10; /* number of iterations */
//	ctl.conf->warmstart = 1;

	createXRef(setpoint, state);

	ctl.x_ref = x_ref;

	// logging
	xVel = state->velocity.x;
	yVel = state->velocity.y;
	zPos = state->position.z;

	/* The current state */
	x[0] = 0.0;
	x[1] = 0.0;
	x[2] = xVel;
	x[3] = yVel;

	//	for (int i = 0; i < MPC_STATES; i++) {
	//		consolePrintf("x[%i] = %f \n", i, (double ) x[i]);
	//	}

	mpc_ctl_solve_problem(&ctl, x); /* solve the MPC problem */

	// use only first input
	u1 = (double) ctl.u_opt[0];
	u2 = (double) ctl.u_opt[1];

#endif

#ifdef OSQP_MPC
	// Solve Problem
	osqp_solve(&workspace);

#endif

//	consolePrintf("u1 = %f, u2 = %f \n --------\n", u1, u2);

	// TODO
	// Roll and Pitch
	float rollRaw = u1;
	float pitchRaw = u2;

	// transform to body frame
	float yawRad = state->attitude.yaw * (float) M_PI / 180;
	attitude->pitch = -(rollRaw * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
	attitude->roll = -(pitchRaw * cosf(yawRad)) + (rollRaw * sinf(yawRad));

	// limit roll/pitch
	attitude->roll = constrain(attitude->roll, -rpLimit, rpLimit);
	attitude->pitch = constrain(attitude->pitch, -rpLimit, rpLimit);

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
void positionControllerMPCResetAll() {

	pidReset(&this.pidZ.pid);

	pidReset(&this.pidVZ.pid);
}

LOG_GROUP_START(posMPC)
LOG_ADD(LOG_FLOAT, input_1, &u1)
LOG_ADD(LOG_FLOAT, input_2, &u2)
LOG_ADD(LOG_FLOAT, x_ref_vel, &xVelRef )
LOG_ADD(LOG_FLOAT, y_ref_vel, &yVelRef )
LOG_ADD(LOG_FLOAT, x_vel, &xVel )
LOG_ADD(LOG_FLOAT, y_vel, &yVel )
LOG_ADD(LOG_FLOAT, z_pos, &zPos )
LOG_GROUP_STOP(posMPC)
