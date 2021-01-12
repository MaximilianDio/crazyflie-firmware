/*
 * horizontal_controller_mpc.c
 *
 *  Created on: Jan 4, 2021
 *      Author: maximilian
 */
#include <math.h>

#include "horizontal_controller_mpc.h"
#include "mpc.h"
#include "num.h"
#include "log.h"
#include "console.h"

// for logging
static float xVelRef = 0.0;
static float yVelRef = 0.0;
static float xVel = 0.0;
static float yVel = 0.0;

// integral action states
static float xIntRef = 0.0;
static float yIntRef = 0.0;
static float xInt = 0.0;
static float yInt = 0.0;

// reference angles calculated by MPC
static double u1;
static double u2;

// Maximum roll/pitch angle permited
static float rpLimit = 20; // SHOULD BE LARGER OR SAME AS IN MPC DESCRIPTION, otherwise MPC will have a hard time

// sampling time
#define DT (float)(1.0f/MPC_RATE)

// initialize reference trajectory (skeleton)
static real_t x_ref[MPC_HOR_STATES] = { 0 };

// define reference trajectory
void createXRef(const setpoint_t *setpoint, const state_t *state) {
	// TODO - change reference trajectory based on MAC forward simulation

	float speedCoefficient = 1;

    // transform to body frame
	float cosyaw = cosf(state->attitude.yaw * (float) M_PI / 180.0f);
	float sinyaw = sinf(state->attitude.yaw * (float) M_PI / 180.0f);
	float bodyvx = speedCoefficient * setpoint->velocity.x;
	float bodyvy = speedCoefficient * setpoint->velocity.y;

	// transform reference velocity to body frame
	xVelRef = (bodyvx * cosyaw - bodyvy * sinyaw);
	yVelRef = (bodyvy * cosyaw + bodyvx * sinyaw);

	// initialize first command
	x_ref[0] = (real_t) xIntRef; // pos_x
	x_ref[1] = (real_t) yIntRef; // pos_y
	x_ref[2] = (real_t) xVelRef; // vel_x
	x_ref[3] = (real_t) yVelRef; // vel_y

	for (int i = 1; i < MPC_HOR; i++) {
		// velocity
		// dampened by decrease_value
		x_ref[i * MPC_STATES + 2] = (real_t) x_ref[(i - 1) * MPC_STATES + 2]; // vel_x
		x_ref[i * MPC_STATES + 3] = (real_t) x_ref[(i - 1) * MPC_STATES + 3]; // vel_y

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

	// update integral action reference
	xIntRef += DT * xVelRef;
	yIntRef += DT * yVelRef;

}

void horizontalControllerMPCInit() {
	u1 = 0.0;
	u2 = 0.0;

}

void horizontalControllerMPCResetAll() {
	// FIXME
	u1 = 0.0;
	u2 = 0.0;

	xInt = 0.0;
	yInt = 0.0;
	xIntRef = 0.0;
	yIntRef = 0.0;
}

void horizontalControllerMPC(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state) {

	horizontalVelocityControllerMPC(thrust, attitude, setpoint, state);

}

void horizontalVelocityControllerMPC(float* thrust, attitude_t *attitude,
		setpoint_t *setpoint, const state_t *state) {
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

	// update integral action states
	xInt += DT * xVel;
	yInt += DT * yVel;

	/* The current state */
	x[0] = xInt;
	x[1] = yInt;
	x[2] = xVel;
	x[3] = yVel;

	mpc_ctl_solve_problem(&ctl, x); /* solve the MPC problem */

	// use only first input
	u1 = (double) ctl.u_opt[0];
	u2 = (double) ctl.u_opt[1];

//	consolePrintf("u[1] = %f, u[2] = %f ", u1, u2);

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

	// for logging (don't want raw roll and pitch)
	u1 = attitude->roll;
	u2 = attitude->pitch;
}

LOG_GROUP_START(MPC_LOGGING)
LOG_ADD((LOG_FLOAT), x_vel ,&xVel)
LOG_ADD((LOG_FLOAT), y_vel ,&yVel)
LOG_ADD((LOG_FLOAT), x_vel_ref ,&xVelRef)
LOG_ADD((LOG_FLOAT), y_vel_ref ,&yVelRef)
LOG_ADD((LOG_FLOAT), x_int_vel ,&xInt)
LOG_ADD((LOG_FLOAT), y_int_vel ,&yInt)
LOG_ADD((LOG_FLOAT), x_int_vel_ref ,&xIntRef)
LOG_ADD((LOG_FLOAT), y_int_vel_ref ,&yIntRef)
LOG_ADD((LOG_FLOAT), u1_roll, &u1)
LOG_ADD((LOG_FLOAT), u2_pitch, &u2)
LOG_GROUP_STOP(MPC_LOGGING)
