#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_mpc.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#include "debug.h"

#define USE_PID
// define which solver to use!
#define MUAO_MPC
#ifdef MUAO_MPC
// muAO-MPC
#include "mpc.h"
#endif

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

// see cascaded PID
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

// control input
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

// reference values
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

// for testing purposes
static double u;

//

void controllerMPCInit(void) {
#ifdef USE_PID
	attitudeControllerInit(ATTITUDE_UPDATE_DT);
	positionControllerInit();
#endif
	u = 0.0;
	// TODO

	DEBUG_PRINT("MPC initialized!\n");
}

bool controllerMPCTest(void) {
	bool pass = true;
#ifdef USE_PID
	pass &= attitudeControllerTest();
#endif

	return pass;
	// TODO
}

static float capAngle(float angle) {
	/*
	 * caps angels to -180 to 180
	 */
	float result = angle;

	while (result > 180.0f) {
		result -= 360.0f;
	}

	while (result < -180.0f) {
		result += 360.0f;
	}

	return result;
}

void controllerMPC(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
#ifdef USE_PID
	// cascaded PID

	// yaw is controlled independently!
	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		// Rate-controlled YAW is moving YAW angle setpoint
		if (setpoint->mode.yaw == modeVelocity) {
			attitudeDesired.yaw += setpoint->attitudeRate.yaw
					* ATTITUDE_UPDATE_DT;
		} else {
			attitudeDesired.yaw = setpoint->attitude.yaw;
		}

		attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
	}

	if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
		positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
	}

	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		// Switch between manual and automatic position control
		if (setpoint->mode.z == modeDisable) {
			// manual z-position control
			actuatorThrust = setpoint->thrust;
		}

		if (setpoint->mode.x == modeDisable
				|| setpoint->mode.y == modeDisable) {
			// manual x,y-position control
			attitudeDesired.roll = setpoint->attitude.roll;
			attitudeDesired.pitch = setpoint->attitude.pitch;
		}

		// control attitude (PID)
		attitudeControllerCorrectAttitudePID(state->attitude.roll,
				state->attitude.pitch, state->attitude.yaw,
				attitudeDesired.roll, attitudeDesired.pitch,
				attitudeDesired.yaw, &rateDesired.roll, &rateDesired.pitch,
				&rateDesired.yaw);

		// For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
		// value. Also reset the PID to avoid error buildup, which can lead to unstable
		// behavior if level mode is engaged later
		if (setpoint->mode.roll == modeVelocity) {
			rateDesired.roll = setpoint->attitudeRate.roll;
			attitudeControllerResetRollAttitudePID();
		}
		if (setpoint->mode.pitch == modeVelocity) {
			rateDesired.pitch = setpoint->attitudeRate.pitch;
			attitudeControllerResetPitchAttitudePID();
		}

		// TODO: Investigate possibility to subtract gyro drift.
		attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y,
				sensors->gyro.z, rateDesired.roll, rateDesired.pitch,
				rateDesired.yaw);

		attitudeControllerGetActuatorOutput(&control->roll, &control->pitch,
				&control->yaw);

		control->yaw = -control->yaw;

		cmd_thrust = control->thrust;
		cmd_roll = control->roll;
		cmd_pitch = control->pitch;
		cmd_yaw = control->yaw;
		r_roll = radians(sensors->gyro.x);
		r_pitch = -radians(sensors->gyro.y);
		r_yaw = radians(sensors->gyro.z);
		accelz = sensors->acc.z;
	}

	if (tiltCompensationEnabled) {
		control->thrust = actuatorThrust
				/ sensfusion6GetInvThrustCompensationForTilt();
	} else {
		control->thrust = actuatorThrust;
	}

	// reset controller if no thrust is given
	if (control->thrust == 0) {
		control->thrust = 0;
		control->roll = 0;
		control->pitch = 0;
		control->yaw = 0;

		cmd_thrust = control->thrust;
		cmd_roll = control->roll;
		cmd_pitch = control->pitch;
		cmd_yaw = control->yaw;

		attitudeControllerResetAllPID();
		positionControllerResetAllPID();

		// Reset the calculated YAW angle for rate control
		attitudeDesired.yaw = state->attitude.yaw;
	}

#endif
	// TODO
#ifdef MUAO_MPC
	// muAO-MPC
	if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
		real_t x[MPC_STATES]; /* current state of the system */
		extern struct mpc_ctl ctl; /* already defined */
		ctl.conf->in_iter = 10; /* number of iterations */

		/* The current state */
		x[0] = state->attitude.pitch;
		x[1] = state->attitude.roll;

		for (int i = 0; i < 2; i++) {
			consolePrintf("x[%i] = %f \n", i, (double ) x[0]);
		}

		/* Solve MPC problem and print the first element of input sequence */
		mpc_ctl_solve_problem(&ctl, x); /* solve the MPC problem */

		u = (double) ctl.u_opt[0];
	}
#endif

	if (RATE_DO_EXECUTE(RATE_25_HZ, tick)) {
		consolePrintf("u = %f \n", u);
	}

}

LOG_GROUP_START(controller) LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll, &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch, &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw, &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate, &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate, &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START( controller) PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
