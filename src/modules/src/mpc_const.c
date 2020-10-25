#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.7933187323742847, 0.05312754600877718, 0.04631196247616864, 0.03966456540125084, 0.03320959003733235, 0.026972127162116683, 0.020978234633577675, 0.01525505324556149, 0.009830927398219938, 0.0047355311197865655, 
0.05312754600877718, 0.7881592776421198, 0.04800363956322075, 0.04122605498431033, 0.03461927647697793, 0.028207719888370918, 0.02201666904000934, 0.016072388141951652, 0.010402238565856835, 0.005034800496126662, 
0.04631196247616864, 0.04800363956322075, 0.7830428432121255, 0.042895192262859594, 0.036126145439156696, 0.02952849327224091, 0.023126692185797573, 0.01694606937519496, 0.011012935414494036, 0.005354701235649179, 
0.03966456540125084, 0.04122605498431033, 0.042895192262859594, 0.7779103022290196, 0.03773689660006635, 0.030940319592089663, 0.024313239334615197, 0.017879981411366222, 0.011665733157758946, 0.0056966556460822625, 
0.03320959003733235, 0.03461927647697793, 0.036126145439156696, 0.03773689660006635, 0.7726895953618486, 0.03244947595589922, 0.025581585982799245, 0.018878276508157717, 0.01236353419367836, 0.00606218408777693, 
0.026972127162116683, 0.028207719888370918, 0.02952849327224091, 0.030940319592089663, 0.03244947595589922, 0.7672935760661207, 0.026937371314686626, 0.01994539317619387, 0.013109441009003563, 0.006452911733367212, 
0.02097823463357767, 0.022016669040009335, 0.023126692185797573, 0.024313239334615197, 0.025581585982799245, 0.026937371314686626, 0.7616175271311978, 0.021086075913052428, 0.013906769973148056, 0.0068705757934352145, 
0.01525505324556149, 0.01607238814195165, 0.01694606937519496, 0.017879981411366222, 0.018878276508157717, 0.01994539317619387, 0.021086075913052425, 0.7555363001539894, 0.014759066083067054, 0.007317033240307069, 
0.009830927398219938, 0.010402238565856835, 0.011012935414494034, 0.011665733157758946, 0.01236353419367836, 0.013109441009003563, 0.013906769973148056, 0.014759066083067053, 0.7489010225808939, 0.007794269064320469, 
0.0047355311197865655, 0.005034800496126662, 0.00535470123564918, 0.0056966556460822625, 0.00606218408777693, 0.006452911733367212, 0.0068705757934352145, 0.007317033240307069, 0.007794269064320469, 0.74153530895553, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
54.46773624522621, 5.918440568374634, 
45.00923230549002, 5.226648655528577, 
36.28874533993925, 4.5503093040347675, 
28.357153578800506, 3.8917944661538275, 
21.268842751994264, 3.2535553435519913, 
15.081947893027106, 2.638133285566231, 
9.858611812628142, 2.0481710882741084, 
5.665261391326775, 1.4864247446050474, 
2.5729029193936817, 0.9557756977350823, 
0.6574377972532279, 0.45924365224479413, 

};
const real_t Bh_T[] = { /* Extended input matrix (used for reference tracking). */
1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 0.0001247782675800015, 0.007920362206999975, 0.00015542674988226502, 0.007409554168628916, 0.00018409861915665045, 0.006931689680722491, 0.00021092135313675143, 0.006484644114387475, 0.00023601420813278083, 0.006066429864453651, 0.0002594887492559518, 0.0056751875124008005, 0.00028144934644709033, 0.0053091775592151585, 
0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 0.0001247782675800015, 0.007920362206999975, 0.00015542674988226502, 0.007409554168628916, 0.00018409861915665045, 0.006931689680722491, 0.00021092135313675143, 0.006484644114387475, 0.00023601420813278083, 0.006066429864453651, 0.0002594887492559518, 0.0056751875124008005, 
0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 0.0001247782675800015, 0.007920362206999975, 0.00015542674988226502, 0.007409554168628916, 0.00018409861915665045, 0.006931689680722491, 0.00021092135313675143, 0.006484644114387475, 0.00023601420813278083, 0.006066429864453651, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 0.0001247782675800015, 0.007920362206999975, 0.00015542674988226502, 0.007409554168628916, 0.00018409861915665045, 0.006931689680722491, 0.00021092135313675143, 0.006484644114387475, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 0.0001247782675800015, 0.007920362206999975, 0.00015542674988226502, 0.007409554168628916, 0.00018409861915665045, 0.006931689680722491, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 0.0001247782675800015, 0.007920362206999975, 0.00015542674988226502, 0.007409554168628916, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 0.0001247782675800015, 0.007920362206999975, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 9.201690631530965e-05, 0.00846638489474484, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 5.699700610196745e-05, 0.009050049898300543, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9562865284559644e-05, 0.00967395224525734, 

};
const real_t E[] = { /* Linear factor (prediction matrix) of 2-sided state constraint. */
0, 

};
const real_t Kx_Ai[] = { /* Prediction component of the state constraint bound. */
0, 

};

const real_t u_lb[] = { /* Left (lower) constraint of the inputs for condensed QP. */
-100, 
-100, 
-100, 
-100, 
-100, 
-100, 
-100, 
-100, 
-100, 
-100, 

};
const real_t u_ub[] = { /* Right (upper) constraint of the inputs for condensed QP. */
100, 
100, 
100, 
100, 
100, 
100, 
100, 
100, 
100, 
100, 

};
const real_t e_lb[] = { /* Left (lower) constraint of the states for condensed QP. */
0, 

};
const real_t e_ub[] = { /* Right (upper) constraint of the states for condensed QP. */
0, 

};

const real_t nu = 0.07665239236336567; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 3.0551287660677424; /* Inverse of gradient Lipschitz constant (1/L) */

/* state dependent variables */
real_t gxoL[MPC_HOR_INPUTS];  /* gradient vector as a function of the current state */
real_t zx_lb[MPC_HOR_MXCONSTRS];  /* mixed constraint lower bound as function of current state */
real_t zx_ub[MPC_HOR_MXCONSTRS];  /* mixed constraint upper bound as function of current state */

/* reference dependent variables */
real_t groL[MPC_HOR_INPUTS];

/* MPC system: state-space and weighting matrices */
const real_t Q[] = {  /* State weighting matrix */
11000.0, 0.0, 
0.0, 29.0, 

};
const real_t R[] = {   /* Input weighting matrix */
0.24, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
11000.0, 0.0, 
0.0, 29.0, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
0, 

};
const real_t Ad[] = {   /* Discrete-time system matrix */
1.0, 0.003869580898102936, 
0.0, 0.9355069850316177, 

};
const real_t Bd[] = {   /* Discrete-time input matrix */
1.9562865284559644e-05, 
0.00967395224525734, 

};
const real_t dt = 0.004;


/* User variables created with appropriate size */
real_t u_opt[MPC_HOR_INPUTS];  /* Optimal input */
real_t l_opt[MPC_HOR_MXCONSTRS];  /* Optimal multiplier */
real_t x_trj[MPC_HOR_STATES + MPC_STATES];  /* State trajectory for u_opt */
real_t u_ini[MPC_HOR_INPUTS];  /* Initial guess input */
real_t l_ini[MPC_HOR_MXCONSTRS];  /* Initial guess multiplier */
real_t ctl_x_ref[MPC_HOR_STATES];  /* Reference for state trajectory */
real_t ctl_u_ref[MPC_HOR_INPUTS];  /* Reference for input trajectory */

/* Always check this declarations match the structure definitions */
struct mpc_conf conf = {1, 1, 0};

struct mpc_qpx qpx = {HoL, gxoL, E, u_lb, u_ub, zx_lb, zx_ub,
		MPC_HOR_INPUTS, MPC_HOR_MXCONSTRS};

struct mpc_sys sys = {Ad, Bd, &dt};

struct mpc_wmx wmx = {Q, R, P,};

struct mpc_lqr lqr = {K, P,};

struct mpc_fgm fgm = {u_ini, gxoL, groL, &(conf.in_iter),
			HoL, GoL, Bh_T, u_lb, u_ub, &nu,
			MPC_HOR, MPC_STATES, MPC_INPUTS, MPC_HOR_INPUTS, MPC_HOR_STATES};

struct mpc_alm alm = {&fgm, l_ini, zx_lb, zx_ub, &(conf.ex_iter),
			&mu,
			E, Kx_Ai, e_lb, e_ub,
			&Linv, 
			MPC_STATES, MPC_MXCONSTRS, MPC_HOR_INPUTS, MPC_HOR_MXCONSTRS};

struct mpc_ctl ctl = {&conf, &qpx, &sys, &wmx, &lqr, &alm,
			u_opt, l_opt, x_trj, 0, 0, u_ini, l_ini};

