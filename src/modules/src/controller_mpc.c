#include "controller_mpc.h"
#include "controller_pid.h"

#include "debug.h"

#include "mpc.h"

void controllerMPCInit(void) {
	controllerPidInit();
	DEBUG_PRINT("MPC initialized!\n");
	// TODO
}

bool controllerMPCTest(void) {
	return controllerPidTest();
	// TODO
}

void controllerMPC(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
	controllerPid(control, setpoint, sensors, state, tick);

	// TODO

	if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
	real_t x[MPC_STATES]; /* current state of the system */
		extern struct mpc_ctl ctl; /* already defined */
		ctl.conf->in_iter = 10; /* number of iterations */
		/* The current state */
		x[0] = 0.1;
		x[1] = -0.5;
		/* Solve MPC problem and print the first element of input sequence */
		mpc_ctl_solve_problem(&ctl, x); /* solve the MPC problem */

		double u = (double) ctl.u_opt[0];
		consolePrintf("10%f",u);
	}

}
