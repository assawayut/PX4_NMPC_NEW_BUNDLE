/* Produced by CVXGEN, 2022-11-04 02:13:58 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double x_ss_0[9];
  double Q[81];
  double R[9];
  double x_ss_1[9];
  double x_ss_2[9];
  double x_ss_3[9];
  double x_ss_4[9];
  double x_ss_5[9];
  double x_ss_6[9];
  double x_ss_7[9];
  double x_ss_8[9];
  double x_ss_9[9];
  double P[81];
  double A[81];
  double B[27];
  double *x_ss[10];
} Params;
typedef struct Vars_t {
  double *x_0; /* 9 rows. */
  double *u_0; /* 3 rows. */
  double *x_1; /* 9 rows. */
  double *u_1; /* 3 rows. */
  double *x_2; /* 9 rows. */
  double *u_2; /* 3 rows. */
  double *x_3; /* 9 rows. */
  double *u_3; /* 3 rows. */
  double *x_4; /* 9 rows. */
  double *u_4; /* 3 rows. */
  double *x_5; /* 9 rows. */
  double *u_5; /* 3 rows. */
  double *x_6; /* 9 rows. */
  double *u_6; /* 3 rows. */
  double *x_7; /* 9 rows. */
  double *u_7; /* 3 rows. */
  double *x_8; /* 9 rows. */
  double *u_8; /* 3 rows. */
  double *x_9; /* 9 rows. */
  double *x[10];
  double *u[9];
} Vars;
typedef struct Workspace_t {
  double *h;
  double *s_inv;
  double *s_inv_z;
  double b[81];
  double q[117];
  double rhs[198];
  double x[198];
  double *s;
  double *z;
  double *y;
  double lhs_aff[198];
  double lhs_cc[198];
  double buffer[198];
  double buffer2[198];
  double KKT[1557];
  double L[2088];
  double d[198];
  double v[198];
  double d_inv[198];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_114597502976[1];
  double quad_600568381440[1];
  double quad_898851794944[1];
  double quad_88433618944[1];
  double quad_240204779520[1];
  double quad_635618762752[1];
  double quad_732753989632[1];
  double quad_427523055616[1];
  double quad_976046530560[1];
  double quad_246140747776[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
