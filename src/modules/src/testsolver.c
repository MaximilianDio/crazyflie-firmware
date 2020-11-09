/* Produced by CVXGEN, 2020-10-25 11:13:40 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.x_0[0] = 0.20319161029830202;
  params.x_0[1] = 0.8325912904724193;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.2909047389129444;
  params.Q[2] = 0;
  params.Q[1] = 0;
  params.Q[3] = 1.510827605197663;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.8929469543476547;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = 1.896293088933438;
  params.Q_final[2] = 0;
  params.Q_final[1] = 0;
  params.Q_final[3] = 1.1255853104638363;
  params.A[0] = -1.171028487447253;
  params.A[1] = -1.7941311867966805;
  params.A[2] = -0.23676062539745413;
  params.A[3] = -1.8804951564857322;
  params.B[0] = -0.17266710242115568;
  params.B[1] = 0.596576190459043;
  params.u_max[0] = 0.5569745652959506;
}
