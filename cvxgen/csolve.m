% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_0 - x_ss_0, Q) + quad_form(u_0, R) + quad_form(x_1 - x_ss_1, Q) + quad_form(u_1, R) + quad_form(x_2 - x_ss_2, Q) + quad_form(u_2, R) + quad_form(x_3 - x_ss_3, Q) + quad_form(u_3, R) + quad_form(x_4 - x_ss_4, Q) + quad_form(u_4, R) + quad_form(x_5 - x_ss_5, Q) + quad_form(u_5, R) + quad_form(x_6 - x_ss_6, Q) + quad_form(u_6, R) + quad_form(x_7 - x_ss_7, Q) + quad_form(u_7, R) + quad_form(x_8 - x_ss_8, Q) + quad_form(u_8, R) + quad_form(x_9 - x_ss_9, P))
%   subject to
%     x_1 == A*x_0 + B*u_0
%     x_2 == A*x_1 + B*u_1
%     x_3 == A*x_2 + B*u_2
%     x_4 == A*x_3 + B*u_3
%     x_5 == A*x_4 + B*u_4
%     x_6 == A*x_5 + B*u_5
%     x_7 == A*x_6 + B*u_6
%     x_8 == A*x_7 + B*u_7
%     x_9 == A*x_8 + B*u_8
%
% with variables
%      u_0   3 x 1
%      u_1   3 x 1
%      u_2   3 x 1
%      u_3   3 x 1
%      u_4   3 x 1
%      u_5   3 x 1
%      u_6   3 x 1
%      u_7   3 x 1
%      u_8   3 x 1
%      x_0   9 x 1
%      x_1   9 x 1
%      x_2   9 x 1
%      x_3   9 x 1
%      x_4   9 x 1
%      x_5   9 x 1
%      x_6   9 x 1
%      x_7   9 x 1
%      x_8   9 x 1
%      x_9   9 x 1
%
% and parameters
%        A   9 x 9
%        B   9 x 3
%        P   9 x 9    PSD
%        Q   9 x 9    PSD
%        R   3 x 3    PSD
%   x_ss_0   9 x 1
%   x_ss_1   9 x 1
%   x_ss_2   9 x 1
%   x_ss_3   9 x 1
%   x_ss_4   9 x 1
%   x_ss_5   9 x 1
%   x_ss_6   9 x 1
%   x_ss_7   9 x 1
%   x_ss_8   9 x 1
%   x_ss_9   9 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.x_ss_9, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2022-11-04 02:13:56 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
