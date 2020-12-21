#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.8111182197426874, 0.0, 0.05353202643339651, 0.0, 0.05108691224645471, 0.0, 0.048641886607932026, 0.0, 0.04619699379203803, 0.0, 
0.0, 0.8111182197426874, 0.0, 0.05353202643339651, 0.0, 0.05108691224645471, 0.0, 0.048641886607932026, 0.0, 0.04619699379203803, 
0.05353202643339651, 0.0, 0.8084524661211792, 0.0, 0.05087713670320198, 0.0, 0.04844288640757387, 0.0, 0.046008724660364875, 0.0, 
0.0, 0.05353202643339651, 0.0, 0.8084524661211792, 0.0, 0.05087713670320198, 0.0, 0.04844288640757387, 0.0, 0.046008724660364875, 
0.05108691224645471, 0.0, 0.05087713670320198, 0.0, 0.8058083960080887, 0.0, 0.04824388620721571, 0.0, 0.04582045552869173, 0.0, 
0.0, 0.05108691224645471, 0.0, 0.05087713670320198, 0.0, 0.8058083960080887, 0.0, 0.04824388620721571, 0.0, 0.04582045552869173, 
0.048641886607932026, 0.0, 0.04844288640757387, 0.0, 0.0482438862072157, 0.0, 0.8031859208549971, 0.0, 0.04563218639701857, 0.0, 
0.0, 0.048641886607932026, 0.0, 0.04844288640757387, 0.0, 0.0482438862072157, 0.0, 0.8031859208549971, 0.0, 0.04563218639701857, 
0.04619699379203803, 0.0, 0.046008724660364875, 0.0, 0.04582045552869173, 0.0, 0.045632186397018576, 0.0, 0.8005849521134849, 0.0, 
0.0, 0.04619699379203803, 0.0, 0.046008724660364875, 0.0, 0.04582045552869173, 0.0, 0.045632186397018576, 0.0, 0.8005849521134849, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
13.518477870360172, 0.0, 32.76133799058582, 0.0, 
0.0, 13.518477870360172, 0.0, 32.76133799058582, 
12.883967043282325, 0.0, 31.330058809899192, 0.0, 
0.0, 12.883967043282325, 0.0, 31.330058809899192, 
12.252042072829461, 0.0, 29.898818417061946, 0.0, 
0.0, 12.252042072829461, 0.0, 29.898818417061946, 
11.622702959001574, 0.0, 28.467642670640323, 0.0, 
0.0, 11.622702959001574, 0.0, 28.467642670640323, 
10.995949701798665, 0.0, 27.03655742920057, 0.0, 
0.0, 10.995949701798665, 0.0, 27.03655742920057, 

};
const real_t Bh_T[] = { /* Extended input matrix (used for reference tracking). */
8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 0.0, 4.280419990516093e-05, 0.0, 0.0017121679962064373, 0.0, 5.99258798672253e-05, 0.0, 0.0017121679962064373, 0.0, 7.704755982928968e-05, 0.0, 0.0017121679962064373, 0.0, 
0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 0.0, 4.280419990516093e-05, 0.0, 0.0017121679962064373, 0.0, 5.99258798672253e-05, 0.0, 0.0017121679962064373, 0.0, 7.704755982928968e-05, 0.0, 0.0017121679962064373, 
0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 0.0, 4.280419990516093e-05, 0.0, 0.0017121679962064373, 0.0, 5.99258798672253e-05, 0.0, 0.0017121679962064373, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 0.0, 4.280419990516093e-05, 0.0, 0.0017121679962064373, 0.0, 5.99258798672253e-05, 0.0, 0.0017121679962064373, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 0.0, 4.280419990516093e-05, 0.0, 0.0017121679962064373, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 0.0, 4.280419990516093e-05, 0.0, 0.0017121679962064373, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 2.5682519943096562e-05, 0.0, 0.0017121679962064373, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.560839981032187e-06, 0.0, 0.0017121679962064373, 

};
const real_t E[] = { /* Linear factor (prediction matrix) of 2-sided state constraint. */
0, 

};
const real_t Kx_Ai[] = { /* Prediction component of the state constraint bound. */
0, 

};

const real_t u_lb[] = { /* Left (lower) constraint of the inputs for condensed QP. */
-40, 
-40, 
-40, 
-40, 
-40, 
-40, 
-40, 
-40, 
-40, 
-40, 

};
const real_t u_ub[] = { /* Right (upper) constraint of the inputs for condensed QP. */
40, 
40, 
40, 
40, 
40, 
40, 
40, 
40, 
40, 
40, 

};
const real_t e_lb[] = { /* Left (lower) constraint of the states for condensed QP. */
0, 

};
const real_t e_ub[] = { /* Right (upper) constraint of the states for condensed QP. */
0, 

};

const real_t nu = 0.06989615200561346; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 15.10282069696279; /* Inverse of gradient Lipschitz constant (1/L) */

/* state dependent variables */
real_t gxoL[MPC_HOR_INPUTS];  /* gradient vector as a function of the current state */
real_t zx_lb[MPC_HOR_MXCONSTRS];  /* mixed constraint lower bound as function of current state */
real_t zx_ub[MPC_HOR_MXCONSTRS];  /* mixed constraint upper bound as function of current state */

/* reference dependent variables */
real_t groL[MPC_HOR_INPUTS];

/* MPC system: state-space and weighting matrices */
const real_t Q[] = {  /* State weighting matrix */
10, 0, 0, 0, 
0, 10, 0, 0, 
0, 0, 50, 0, 
0, 0, 0, 50, 

};
const real_t R[] = {   /* Input weighting matrix */
0.05, 0.0, 
0.0, 0.05, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
2418.7741998095003, 0.0, 413.1404438342648, 0.0, 
0.0, 2418.7741998095003, 0.0, 413.1404438342648, 
413.1404438342648, 0.0, 1022.2277442255635, 0.0, 
0.0, 413.1404438342648, 0.0, 1022.2277442255635, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
-13.734894307930935, 0.0, -33.2216079891525, 0.0, 
0.0, -13.734894307930935, 0.0, -33.2216079891525, 

};
const real_t Ad[] = {   /* Discrete-time system matrix */
1.0, 0.0, 0.01, 0.0, 
0.0, 1.0, 0.0, 0.01, 
0.0, 0.0, 1.0, 0.0, 
0.0, 0.0, 0.0, 1.0, 

};
const real_t Bd[] = {   /* Discrete-time input matrix */
8.560839981032187e-06, 0.0, 
0.0, 8.560839981032187e-06, 
0.0017121679962064373, 0.0, 
0.0, 0.0017121679962064373, 

};
const real_t dt = 0.01;


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

