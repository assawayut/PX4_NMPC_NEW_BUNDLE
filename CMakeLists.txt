############################################################################
#
#   Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(AttitudeControl)

px4_add_module(
	MODULE modules__mc_att_control
	MAIN mc_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		mc_att_control_main.cpp
		mc_att_control.hpp
		mpc_solver/acado_auxiliary_functions.c
		mpc_solver/acado_common.h
		mpc_solver/acado_integrator.c
		mpc_solver/acado_qpoases_interface.cpp
		mpc_solver/acado_qpoases_interface.hpp
		mpc_solver/acado_solver.c
		mpc_solver/qpoases/SRC/QProblemB.cpp
		mpc_solver/qpoases/SRC/Bounds.cpp
		mpc_solver/qpoases/SRC/Constraints.cpp
		mpc_solver/qpoases/SRC/CyclingManager.cpp
		mpc_solver/qpoases/SRC/Indexlist.cpp
		mpc_solver/qpoases/SRC/MessageHandling.cpp
		mpc_solver/qpoases/SRC/QProblem.cpp
		mpc_solver/qpoases/SRC/SubjectTo.cpp
		mpc_solver/qpoases/SRC/Utils.cpp
		mpc_solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp

#		mpc_solver_shorter/acado_auxiliary_functions.c
#		mpc_solver_shorter/acado_common.h
#		mpc_solver_shorter/acado_integrator.c
#		mpc_solver_shorter/acado_qpoases_interface.cpp
#		mpc_solver_shorter/acado_qpoases_interface.hpp
#		mpc_solver_shorter/acado_solver.c
#		mpc_solver_shorter/qpoases/SRC/QProblemB.cpp
#		mpc_solver_shorter/qpoases/SRC/Bounds.cpp
#		mpc_solver_shorter/qpoases/SRC/Constraints.cpp
#		mpc_solver_shorter/qpoases/SRC/CyclingManager.cpp
#		mpc_solver_shorter/qpoases/SRC/Indexlist.cpp
#		mpc_solver_shorter/qpoases/SRC/MessageHandling.cpp
#		mpc_solver_shorter/qpoases/SRC/QProblem.cpp
#		mpc_solver_shorter/qpoases/SRC/SubjectTo.cpp
#		mpc_solver_shorter/qpoases/SRC/Utils.cpp
#		mpc_solver_shorter/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp

#		mpc_least_square_engl/matrix.cpp
#		mpc_least_square_engl/mpc.cpp
#		mpc_least_square_engl/konfig.h
#		mpc_least_square_engl/matrix.h
#		mpc_least_square_engl/mpc.h
#		mpc_least_square_engl/mpc_least_square_engl.ino
#		cvxgen/ldl.c
#		cvxgen/matrix_support.c
#		cvxgen/solver.c
#		cvxgen/util.c
	DEPENDS
		AttitudeControl
		mathlib
		px4_work_queue
	INCLUDES
		mpc_solver
		mpc_solver/qpoases/INCLUDE
		mpc_solver/qpoases/SRC

#		mpc_solver_shorter
#		mpc_solver_shorter/qpoases/INCLUDE
#		mpc_solver_shorter/qpoases/SRC

#		mpc_least_square_engl
#		cvxgen
	)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


