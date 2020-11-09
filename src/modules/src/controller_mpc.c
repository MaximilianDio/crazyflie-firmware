#include "controller_mpc.h"
#include "controller_pid.h"

#include "debug.h"
#include "log.h"

#define USE_PID
// define which solver to use!
#define MUAO_MPC
#ifdef MUAO_MPC
// muAO-MPC
#include "mpc.h"
#endif
#ifdef CVXGEN_MPC
// CVXGEN
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#endif

// for testing purposes
static double u;

#ifdef CVXGEN_MPC
// define problem online!!! CVXGEN
void load_default_data(void) {
	params.x_0[0] = 1.0;
	params.x_0[1] = 1.0;
	/* Make this a diagonal PSD matrix, even though it's not diagonal. */
	params.Q[0] = 1.0;
	params.Q[2] = 0;
	params.Q[1] = 0;
	params.Q[3] = 1.0;
	/* Make this a diagonal PSD matrix, even though it's not diagonal. */
	params.R[0] = 1.0;
	/* Make this a diagonal PSD matrix, even though it's not diagonal. */
	params.Q_final[0] = 1.0;
	params.Q_final[2] = 0;
	params.Q_final[1] = 0;
	params.Q_final[3] = 1.0;
	params.A[0] = 1.0;
	params.A[1] = 0.0095;
	params.A[2] = 0.0;
	params.A[3] = 0.9048;
	params.B[0] = 0.0001;
	params.B[1] = 0.0190;
	params.u_max[0] = 100;
}
#endif

void controllerMPCInit(void) {
#ifdef USE_PID
	controllerPidInit();
#endif
	u = 0.0;
	// TODO
#ifdef CVXGEN_MPC
	// CVXGEN
	set_defaults();
	setup_indexing();
	load_default_data();
	/* Solve problem instance for the record. */
	settings.verbose = 0;
	settings.max_iters = 10;  // reduce the maximum iteration count, from 25.
	settings.eps = 0.1;  // reduce the required objective tolerance, from 1e-6.
	settings.eps = 0.1;  // reduce the required residual tolerances, from 1e-4.
#endif

	DEBUG_PRINT("MPC initialized!\n");
}

bool controllerMPCTest(void) {
#ifdef USE_PID
	return controllerPidTest();
#else
	return 1;
#endif
	// TODO
}

void controllerMPC(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
#ifdef USE_PID
	controllerPid(control, setpoint, sensors, state, tick);
#endif
	// TODO
#ifdef CVXGEN_MPC
	// CVXGEN
	if (RATE_DO_EXECUTE(RATE_25_HZ, tick)) {
		params.x_0[0] = 1.0;
		params.x_0[1] = -0.5;

		solve();
		u = *(vars.u_0);
	}
#endif
#ifdef MUAO_MPC
	// muAO-MPC
	if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
		real_t x[MPC_STATES]; /* current state of the system */
		extern struct mpc_ctl ctl; /* already defined */
		ctl.conf->in_iter = 10; /* number of iterations */

		/* The current state */
		x[0] = state->attitude.pitch;
		x[1] = state->attitude.roll;

		for (int i = 0; i < 2; i++){
			consolePrintf("x[%i] = %f \n",i,(double) x[0]);
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

LOG_GROUP_START(max)
LOG_ADD(LOG_FLOAT, control, &u)
LOG_GROUP_STOP(max)
