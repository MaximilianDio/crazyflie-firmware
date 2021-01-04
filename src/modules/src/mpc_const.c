#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.9180604796151275, 0.0, 0.021515495969922773, 0.0, 0.02115527354103325, 0.0, 0.020795068626696066, 0.0, 0.020434889984187393, 0.0, 
0.0, 0.9180604796151275, 0.0, 0.021515495969922773, 0.0, 0.02115527354103325, 0.0, 0.020795068626696066, 0.0, 0.020434889984187393, 
0.021515495969922773, 0.0, 0.9176047311162008, 0.0, 0.021061745248628493, 0.0, 0.0207035205973715, 0.0, 0.020345313460666842, 0.0, 
0.0, 0.021515495969922773, 0.0, 0.9176047311162008, 0.0, 0.021061745248628493, 0.0, 0.0207035205973715, 0.0, 0.020345313460666842, 
0.021155273541033248, 0.0, 0.021061745248628493, 0.0, 0.9171529694152628, 0.0, 0.020611972568046928, 0.0, 0.02025573693714629, 0.0, 
0.0, 0.021155273541033248, 0.0, 0.021061745248628493, 0.0, 0.9171529694152628, 0.0, 0.020611972568046928, 0.0, 0.02025573693714629, 
0.020795068626696066, 0.0, 0.0207035205973715, 0.0, 0.020611972568046928, 0.0, 0.9167051769977614, 0.0, 0.02016616041362574, 0.0, 
0.0, 0.020795068626696066, 0.0, 0.0207035205973715, 0.0, 0.020611972568046928, 0.0, 0.9167051769977614, 0.0, 0.02016616041362574, 
0.020434889984187393, 0.0, 0.020345313460666842, 0.0, 0.02025573693714629, 0.0, 0.02016616041362574, 0.0, 0.9162613363491443, 0.0, 
0.0, 0.020434889984187393, 0.0, 0.020345313460666842, 0.0, 0.02025573693714629, 0.0, 0.02016616041362574, 0.0, 0.9162613363491443, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
5.695416022826177, 0.0, 12.805101339274092, 0.0, 
0.0, 5.695416022826177, 0.0, 12.805101339274092, 
5.578734853866465, 0.0, 12.594123166698553, 0.0, 
0.0, 5.578734853866465, 0.0, 12.594123166698553, 
5.4625651578573695, 0.0, 12.383152666217272, 0.0, 
0.0, 5.4625651578573695, 0.0, 12.383152666217272, 
5.34690693479889, 0.0, 12.17219495255976, 0.0, 
0.0, 5.34690693479889, 0.0, 12.17219495255976, 
5.2317601846910256, 0.0, 11.961255140455515, 0.0, 
0.0, 5.2317601846910256, 0.0, 11.961255140455515, 

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
-30, 
-30, 
-30, 
-30, 
-30, 
-30, 
-30, 
-30, 
-30, 
-30, 

};
const real_t u_ub[] = { /* Right (upper) constraint of the inputs for condensed QP. */
30, 
30, 
30, 
30, 
30, 
30, 
30, 
30, 
30, 
30, 

};
const real_t e_lb[] = { /* Left (lower) constraint of the states for condensed QP. */
0, 

};
const real_t e_ub[] = { /* Right (upper) constraint of the states for condensed QP. */
0, 

};

const real_t nu = 0.027375078570596786; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 2.987282508196797; /* Inverse of gradient Lipschitz constant (1/L) */

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
0, 0, 30, 0, 
0, 0, 0, 30, 

};
const real_t R[] = {   /* Input weighting matrix */
0.3, 0.0, 
0.0, 0.3, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
2246.2774129920904, 0.0, 1011.6497209952381, 0.0, 
0.0, 2246.2774129920904, 0.0, 1011.6497209952381, 
1011.6497209952381, 0.0, 2282.387669527364, 0.0, 
0.0, 1011.6497209952381, 0.0, 2282.387669527364, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
-5.709899541911311, 0.0, -12.826018371454824, 0.0, 
0.0, -5.709899541911311, 0.0, -12.826018371454824, 

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

