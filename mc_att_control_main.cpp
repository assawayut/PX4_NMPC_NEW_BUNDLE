/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"
#include <iostream>

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <matrix/math.hpp>

///*include <px4.h>*/
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <math.h>
#include "matrix.h"
#include <chrono>


using namespace matrix;
//using namespace Eigen;

Vector3f _att_ctrl;



MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)

{

    _vehicle_attitude.q[0] = 1.f;
    _vehicle_attitude_setpoint.q_d[0] = 1.f;
	parameters_updated();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
					      _param_mc_yaw_weight.get());

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return math::interpolate(throttle_stick_input, 0.f, 1.f, _param_mpc_manthr_min.get(), _param_mpc_thr_max.get());

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		return math::interpolateN(throttle_stick_input, {_param_mpc_manthr_min.get(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
	}
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
//	const float yaw = Eulerf(Quatf(_vehicle_attitude.q)).psi();
    const float yaw = Eulerf(q).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f) > 0.05f
		   || _param_mc_airmode.get() == 2) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_x_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_y_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_x_input_filter.update(_manual_control_setpoint.x * _man_tilt_max);
	_man_y_input_filter.update(_manual_control_setpoint.y * _man_tilt_max);
	const float x = _man_x_input_filter.getState();
	const float y = _man_y_input_filter.getState();

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* modify roll/pitch only if we're a VTOL */
	if (_vtol) {
		// Construct attitude setpoint rotation matrix. Modify the setpoints for roll
		// and pitch such that they reflect the user's intention even if a large yaw error
		// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
		// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
		// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
		// heading of the vehicle.
		// However there's also a coupling effect that causes oscillations for fast roll/pitch changes
		// at higher tilt angles, so we want to avoid using this on multicopters.
		// The effect of that can be seen with:
		// - roll/pitch into one direction, keep it fixed (at high angle)
		// - apply a fast yaw rotation
		// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

		// calculate our current yaw error
		float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

		// compute the vector obtained by rotating a z unit vector by the rotation
		// given by the roll and pitch commands of the user
		Vector3f zB = {0.0f, 0.0f, 1.0f};
		Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
		Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

		// transform the vector into a new frame which is rotated around the z axis
		// by the current yaw error. this vector defines the desired tilt when we look
		// into the direction of the desired heading
		Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

		// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
		// R_tilt is computed from_euler; only true if cos(roll) not equal zero
		// -> valid if roll is not +-pi/2;
		attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
		attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
	}

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(math::constrain(_manual_control_setpoint.z, 0.f, 1.f));
//        cout << "z: " << _manual_control_setpoint.z << endl;
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

	// update attitude controller setpoint immediately
	_attitude_control.setAttitudeSetpoint(q_sp, attitude_setpoint.yaw_sp_move_rate);
	_thrust_setpoint_body = Vector3f(attitude_setpoint.thrust_body);
	_last_attitude_setpoint = attitude_setpoint.timestamp;

}


Eigen::Vector3f
MulticopterAttitudeControl::calculateS1PrincipleBundleMap(Eigen::Vector4f quat){
    Eigen::Vector4f Q = h_real*quat;

    float p_1 = 2*(Q[0]*Q[2]+Q[1]*Q[3]);
    float p_2 = 2*(Q[2]*Q[3]-Q[0]*Q[1]);
    float p_3 = (Q[0]*Q[0]-Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3]);

    Eigen::Vector3f p(p_1, p_2, p_3);

//    cout << "p: " << p << endl;

    return p;

}

Eigen::Vector4f MulticopterAttitudeControl::getFeedbackDynamicExtension(Eigen::Vector3f p, Eigen::Vector3f w, float w_0){
    float p_norm = sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
    Eigen::Matrix<float, 4, 4> Feedback;
    Eigen::Vector4f wControl(w_0, w[0], w[1], w[2]);
    Feedback << 0, p[0]/(2*(p_norm+p[2])), p[1]/(2*(p_norm+p[2])), 0.5f,
                p[0], 0, p[2], -p[1],
                p[1], -p[2], 0, p[0],
                p[2], p[1], -p[0], 0;

    Eigen::Matrix<float, 4, 4> Feedback_inv = Feedback.inverse();
    Eigen::Vector4f dynExt = Feedback_inv * wControl;

//    cout<<"F: "<<Feedback<<endl;
//    cout<<"F_inv: "<<Feedback_inv<<endl;


    return dynExt;
}

void
MulticopterAttitudeControl::hIntgration(float u0, float dt){
//    float uInt = dt*(u_old + u0)*0.5f;
    h_desired = h_old_desired * exp(0.5f*u0*dt);

//    h = h_old + (h_old*u0)/2*dt;

    f_desired = h_desired*h_desired;

    if (f_desired>0.72f*f_max_desired){
        f_desired = 0.72f*f_max_desired;
    }
    if (f_desired<0.70f*f_max_desired){
        f_desired = 0.70f*f_max_desired;
    }

    h_old_desired = sqrt(f_desired);

}

void
MulticopterAttitudeControl::hIntregationReal(float u0, float dt){
//    h_real = h_old_real + h_old_real*u0/2*dt;
//    f_real = h_real*h_real;
//    h_old_real = h_real;

        h_real = sqrt(-1*_thrust_setpoint_body(2)*1.5f*9.81f);

}

Eigen::Vector3f
MulticopterAttitudeControl::bodyAngVelSp(Eigen::Vector4f dynExt, Eigen::Vector4f quat){
    Eigen::Vector4f qC (quat[0], -quat[1], -quat[2], -quat[3]);
    Eigen::Vector3f spatialAngVel(dynExt[1], dynExt[2], dynExt[3]);
    Eigen::Matrix<float, 3, 3> mapQuat;
    mapQuat << qC[0]*qC[0]-qC[1]*qC[1]-qC[2]*qC[2]-qC[3]*qC[3], 2*(qC[1]*qC[2]-qC[0]*qC[3]), 2*(qC[1]*qC[3]+qC[0]*qC[2]),
             2*(qC[1]*qC[2]+qC[0]*qC[3]), qC[0]*qC[0]-qC[1]*qC[1]+qC[2]*qC[2]-qC[3]*qC[3], 2*(qC[2]*qC[3]-qC[0]*qC[1]),
            2*(qC[1]*qC[3]-qC[0]*qC[2]), 2*(qC[2]*qC[3]+qC[0]*qC[1]), qC[0]*qC[0]-qC[1]*qC[1]-qC[2]*qC[2]+qC[3]*qC[3];

//    cout << "MapQuat: " << mapQuat << endl;

    Eigen::Vector3f Omega = mapQuat * spatialAngVel;

    return Omega;
}

// Solve ARE disable
//Eigen::MatrixXd MulticopterAttitudeControl::solveCare(Eigen::MatrixXd Q, Eigen::MatrixXd R)
//{
////    Eigen::MatrixXd A(NX,NX); Eigen::MatrixXd B(NX,NU);
////
////     A << 0, 0, 0, 1, 0, 0, 0, 0, 0,
////    0, 0, 0, 0, 1, 0, 0, 0, 0,
////    0, 0, 0, 0, 0, 1, 0, 0, 0,
////    0, 0, 0, 0, 0, 0, 1/massQuad, 0, 0,
////    0, 0, 0, 0, 0, 0, 0, 1/massQuad, 0,
////    -g/_vehicle_odometry.position[0], 0, 0, 0, 0, 0, 0, 0, 1/massQuad,
////    0, 0, 0, 0, 0, 0, 0, 0, 0,
////    0, 0, 0, 0, 0, 0, 0, 0, 0,
////    0, 0, 0, 0, 0, 0, 0, 0, 0;
////
////     B << 0, 0, 0,
////    0, 0, 0,
////    0, 0, 0,
////    0, 0, 0,
////    0, 0, 0,
////    0, 0, 0,
////    1, 0, 0,
////    0, 1, 0,
////    0, 0, 1;
//////     R.inverse(); B.transpose();
////
////
////     Eigen::MatrixXd G;
////     G = B*R.inverse()*B.transpose();
////
////    Eigen::MatrixXd z11 = A;
////    Eigen::MatrixXd z12 = -1.0 * G;
////    Eigen::MatrixXd z21 = -1.0 * Q;
////    Eigen::MatrixXd z22 = -1.0 * A.transpose();
////
////    Eigen::MatrixXd Z;
////    Z.resize(z11.rows() + z21.rows(), z11.cols() + z12.cols());
////    Z << z11, z12, z21, z22;
////
////    int n = A.cols();
////    Eigen::MatrixXd U(2 * n, 2 * n);  // Orthogonal matrix from Schur decomposition
////    Eigen::VectorXd WR(2 * n);
////    Eigen::VectorXd WI(2 * n);
//////    lapack_int sdim = 0;  // Number of eigenvalues for which sort is true
//////    lapack_int info;
//////    info = LAPACKE_dgees(LAPACK_COL_MAJOR,  // Eigen default storage order
//////                         'V',               // Schur vectors are computed
//////                         'S',               // Eigenvalues are sorted
//////                         select_lhp,        // Ordering callback
//////                         Z.rows(),          // Dimension of test matrix
//////                         Z.data(),          // Pointer to first element
//////                         Z.rows(),          // Leading dimension (column stride)
//////                         &sdim,             // Number of eigenvalues sort is true
//////                         WR.data(),         // Real portion of eigenvalues
//////                         WI.data(),         // Complex portion of eigenvalues
//////                         U.data(),          // Orthogonal transformation matrix
//////                         Z.rows());         // Dimension of Z
////
////    Eigen::MatrixXd U11 = U.block(0, 0, n, n).transpose();
////    Eigen::MatrixXd U21 = U.block(n, 0, n, n).transpose();
////
////    return U11.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(U21).transpose();
//}

void MulticopterAttitudeControl::initAcado(){
//
////    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
////    memset(&acadoVariables, 0, sizeof(acadoVariables));
////
////
////    // State, control, reference respectively
////    for(int i = 0; i<NX*(NH+1); ++i){
////        acadoVariables.x[i] = 0.0;
////        if (i%NX==8){
////            acadoVariables.x[i] = 1.0;
////        }
////    }
////    for(int i=0; i<NU*NH; ++i) {
////        acadoVariables.u[i] = 0.0;
////    }
////    for(int i=0; i<NY*NH; ++i){
////        acadoVariables.y[i] = 0.0;
////    }
////    for(int i=0; i<NYN; ++i){
////        acadoVariables.yN[i] = 0.0;
////    }
////
////    // Weight
////    for (int i=0; i<NY; ++i){
////        acadoVariables.W[i*(NY+1)] = 10;
////    }
////    for (int i=0; i<NX; ++i){
////        acadoVariables.WN[i*(NX+1)] = 10;
////    }
////
////    acado_initializeSolver();
//
}

Eigen::Vector3f
MulticopterAttitudeControl::runAcado(Eigen::Vector3f position, Eigen::Vector3f velocity, Eigen::Vector3f p,
                                     Eigen::Vector3f xd_, Eigen::Vector3f vd_, Eigen::Vector3f pd_){

    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    acado_initializeSolver();

    // State, control, reference respectively
    for(int i = 0; i<NX*(NH+1); ++i){
        acadoVariables.x[i] = 0.0;
        if (i%NX==8){
            acadoVariables.x[i] = h_real*h_real;
        }
    }

    for(int i=0; i<(NY)*(NY)*NH; ++i){
        acadoVariables.W[i] = 0;
    }
    for(int i=0; i<NX*NX; ++i){
        acadoVariables.WN[i] = 0;
    }

    for(int i=0; i<NH; i++){
        acadoVariables.W[13*0 + 144*i] = 100;
        acadoVariables.W[13*1 + 144*i] = 100;
        acadoVariables.W[13*2 + 144*i] = 100;
        acadoVariables.W[13*3 + 144*i] = 100;
        acadoVariables.W[13*4 + 144*i] = 100;
        acadoVariables.W[13*5 + 144*i] = 100;
        acadoVariables.W[13*6 + 144*i] = 10;
        acadoVariables.W[13*7 + 144*i] = 10;
        acadoVariables.W[13*8 + 144*i] = 10;
        acadoVariables.W[13*9 + 144*i] = 0.5;
        acadoVariables.W[13*10 + 144*i] = 0.5;
        acadoVariables.W[13*11 + 144*i] = 0.5;
    }

    for (int i=0; i<NX; i++){
        acadoVariables.WN[(NX+1)*i] = 10;
    }
    acadoVariables.WN[0] = 100;
    acadoVariables.WN[10] = 100;
    acadoVariables.WN[20] = 100;


    for(int i=0; i<NY*NH; ++i){
        acadoVariables.y[i] = 0.0;
    }
    for(int i=0; i<NYN; ++i){
        acadoVariables.yN[i] = 0.0;
    }

    // Weight

//    for(int i=0; i<NY; ++i){
//        acadoVariables.W[i*(NY+1)] = 10;
//    }
//
//    for(int i=0; i<NYN; ++i){
//        acadoVariables.WN[i*(NYN+1)] = 10;
//    }
//
//    Eigen::MatrixXd Wx; Eigen::MatrixXd Wu;
//    Wx = Eigen::MatrixXd::Identity(NX,NX); Wx = Wx*10;
//    Wu = Eigen::MatrixXd::Identity(NU,NU); Wu = Wu*10;
//
//    Eigen::MatrixXd P_final(NX,NX);
//    P_final = solveCare(Wx, Wu);
//
//    for (int i=0; i<NX; ++i){
//        for(int j=0; j<NX; ++j){
//            acadoVariables.WN[i*NX+j] = P_final(i,j);
//        }
//    }

        acadoVariables.x0[0] = position[0];
        acadoVariables.x0[1] = position[1];
        acadoVariables.x0[2] = position[2];
        acadoVariables.x0[3] = velocity[0];
        acadoVariables.x0[4] = velocity[1];
        acadoVariables.x0[5] = velocity[2];
        acadoVariables.x0[6] = p[0];
        acadoVariables.x0[7] = p[1];
        acadoVariables.x0[8] = p[2];

    // Set reference
    for (int i=0; i<NY*NH; ++i){
        if(i%NY==0){
            acadoVariables.y[i] = xd_[0];
        }
        if(i%NY==1){
            acadoVariables.y[i] = xd_[1];
        }
        if(i%NY==2){
            acadoVariables.y[i] = xd_[2];
        }
        if(i%NY==3){
            acadoVariables.y[i] = vd_[0];
        }
        if(i%NY==4){
            acadoVariables.y[i] = vd_[1];
        }
        if(i%NY==5){
            acadoVariables.y[i] = vd_[2];
        }
        if(i%NY==6) {
            acadoVariables.y[i] = pd_[0];
        }
        if(i%NY==7){
            acadoVariables.y[i] = pd_[1];
        }
        if(i%NY==8){
            acadoVariables.y[i] = pd_[2];
        }
        if(i%NY==9){
            acadoVariables.y[i] = +0.1f*(position[0]-xd_[0])+0.01f*(velocity[0]-vd_[0])+0.01f*(p[0]-pd_[0]);
//            acadoVariables.y[i] = 0.0f;
        }
        if(i%NY==10){
            acadoVariables.y[i] = +0.1f*(position[1]-xd_[1])+0.01f*(velocity[1]-vd_[1])+0.01f*(p[1]-pd_[1]);
//            acadoVariables.y[i] = 0.0f;
        }
        if(i%NY==11){
            acadoVariables.y[i] = +0.05f*(position[2]-xd_[2])+0.01f*(velocity[2]-vd_[2])+0.01f*(p[2]-pd_[2]);
//            acadoVariables.y[i] = 0.0f;
        }
    }
    for (int i=0; i<NYN; ++i){
        if(i%NYN==0){
            acadoVariables.yN[i] = xd_[0];
        }
        if(i%NYN==1){
            acadoVariables.yN[i] = xd_[1];
        }
        if(i%NYN==2){
            acadoVariables.yN[i] = xd_[2];
        }
        if(i%NYN==3){
            acadoVariables.yN[i] = vd_[0];
        }
        if(i%NYN==4){
            acadoVariables.yN[i] = vd_[1];
        }
        if(i%NYN==5){
            acadoVariables.yN[i] = vd_[2];
        }
        if(i%NYN==6) {
            acadoVariables.yN[i] = pd_[0];
        }
        if(i%NYN==7){
            acadoVariables.yN[i] = pd_[1];
        }
        if(i%NYN==8){
            acadoVariables.yN[i] = pd_[2];
        }
    }

//    acado_initializeNodesByForwardSimulation();

//    acado_timer timer_;
//    real_t t2;
//    acado_tic( &timer_ );

    acado_preparationStep();

    for (int i=0; i<NUM_STEPS; ++i){
//        acado_feedbackStep();
        acado_feedbackStep();

//        for (int j = 0; j < NX; ++j){
//            acadoVariables.x0[ j ] = acadoVariables.x[NX + j];
//        }
//        acado_shiftStates(2, 0, 0);
//        acado_shiftControls(0);
//        acado_initializeNodesByForwardSimulation();
        acado_preparationStep();

    }

//    t2 = acado_toc( &timer_ );

    Eigen::Vector3f controlInput(acadoVariables.u[0], acadoVariables.u[1], acadoVariables.u[2]);

//
//    acado_printDifferentialVariables();
//    acado_printControlVariables();

//    float a = acado_getObjective();
//    cout << "obj: " << a << endl;

//    cout << "Calculation time: " << t2 << endl;

    return controlInput;
}

void MulticopterAttitudeControl::loadData(){
//    Eigen::MatrixXd A(stateSize, stateSize);
//    A.setZero();
//    Eigen::MatrixXd B(stateSize, controlSize);
//    B.setZero();
//
//    A(0,3) = 1;
//    A(1,4) = 1;
//    A(2,5) = 1;
//    A(3,6) = 1/massQuad;
//    A(4,7) = 1/massQuad;
//    A(5,0) = -g/_vehicle_odometry.position[0];
//    A(5,8) = 1/massQuad;
//
//    B(6,0) = 1;
//    B(7,1) = 1;
//    B(8,2) = 1;
//
//    // Linearize
//
//    Ad = (sampling_time*A).exp();
//
//    Eigen::MatrixXd integral_exp_A;
//    integral_exp_A = Eigen::MatrixXd::Zero(stateSize, stateSize);
//    const int count_integral_A = 100;
//
//    for (int i = 0; i < count_integral_A; i++) {
//        integral_exp_A += (A * sampling_time * i / count_integral_A).exp()
//                          * sampling_time / count_integral_A;
//    }
//
//    Bd = integral_exp_A * B;
//
//    for (int i=0; i<stateSize; ++i){
//        for (int j=0; j<stateSize; ++j){
//            params.A[i*stateSize+j] = A(i,j);
//        }
//        for (int j=0; j<controlSize; ++j){
//            params.B[i*stateSize+j] = B(i,j);
//        }
//    }
//
//    set_defaults();
//    setup_indexing();
//
//    settings.verbose = 0;
}
//
void MulticopterAttitudeControl::setGain(Eigen::Vector3f p){
//    Eigen::MatrixXd Q (stateSize,stateSize);
//    Eigen::MatrixXd R (controlSize, controlSize);
//    Eigen::MatrixXd P (stateSize, stateSize);
//
//    Q.setZero(); P.setZero(); R.setZero();
//
//    Q(0,0) = 10; Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10; Q(4,4) = 10;
//    Q(5,5) = 10; Q(6,6) = 10; Q(7,7) = 10; Q(8,9) = 10; Q(9,9) = 10;
//
//    R(0,0) = 10; R(1,1) = 10; R(2,2) = 10;
//
//    // Terminal Cost
//    P = Q;
//    for (int i=0; i<10; i++){
//        Eigen::MatrixXd temp = (Bd.transpose() * P * Bd + R);
//        P = Ad.transpose() * P * Ad
//                  - (Ad.transpose() * P * Bd) * temp.inverse()
//                    * (Bd.transpose() * P * Ad) + Q;
//    }
//
//    // Set matrix params
//    for(int i=0; i<stateSize; ++i){
//        for (int j=0; j<stateSize; ++j){
//            params.A[i*stateSize+j] = Ad(i,j);
//            params.Q[i*stateSize+j] = Q(i,j);
//            params.P[i*stateSize+j] = P(i,j);
//        }
//    }
//
//    for(int i=0; i<stateSize; ++i){
//        for (int j=0; j<controlSize; ++j){
//            params.B[i*stateSize+j] = Bd(i,j);
//        }
//    }
//
//    for(int i=0; i<controlSize; ++i){
//        for (int j=0; j<controlSize; ++j){
//            params.R[i*stateSize+j] = R(i,j);
//        }
//    }
//
//    float x_ref = 10; float y_ref = 10; float z_ref = -10; float vx_ref = 0; float vy_ref = 0; float vz_ref = 0;
//    float p1_ref = 0; float p2_ref = 0; float p3_ref = 1;
//
//    vector<float> reference_trajectory;
//
//    reference_trajectory = {x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref, p1_ref, p2_ref, p3_ref};
//
//    for (int i = 1; i < stateSize; ++i){
//        params.x_ss_0[i] = reference_trajectory[i];
//        params.x_ss_1[i] = reference_trajectory[i];
//        params.x_ss_2[i] = reference_trajectory[i];
//        params.x_ss_3[i] = reference_trajectory[i];
//        params.x_ss_4[i] = reference_trajectory[i];
//        params.x_ss_5[i] = reference_trajectory[i];
//        params.x_ss_6[i] = reference_trajectory[i];
//        params.x_ss_7[i] = reference_trajectory[i];
//        params.x_ss_8[i] = reference_trajectory[i];
//        params.x_ss_9[i] = reference_trajectory[i];
//    }
//
//     vars.x_0[0] = _vehicle_odometry.position[0];
//    vars.x_0[1] = _vehicle_odometry.position[1];
//    vars.x_0[2] = _vehicle_odometry.position[2];
//    vars.x_0[3] = _vehicle_odometry.velocity[0];
//    vars.x_0[4] = _vehicle_odometry.velocity[1];
//    vars.x_0[5] = _vehicle_odometry.velocity[2];
//    vars.x_0[6] = p[0];
//    vars.x_0[7] = p[1];
//    vars.x_0[8] = p[2];
//
//    int solver_status = solve();
//    controlInputW << vars.u_0[0], vars.u_0[1], vars.u_0[2];
//
//    std::cout << "Status: " << solver_status << std::endl;
//
}

void MulticopterAttitudeControl::mpcLast()
{
//
//
//
//    Eigen::MatrixXd A(stateSize, stateSize);
//    A.setZero();
//    Eigen::MatrixXd B(stateSize, controlSize);
//    B.setZero();
//
//    A(0,3) = 1;
//    A(1,4) = 1;
//    A(2,5) = 1;
//    A(3,6) = 1/massQuad;
//    A(4,7) = 1/massQuad;
//    A(5,0) = -g/_vehicle_odometry.position[0];
//    A(5,8) = 1/massQuad;
//
//    B(6,0) = 1;
//    B(7,1) = 1;
//    B(8,2) = 1;
//
//    // Linearize
//
//    Ad = (sampling_time*A).exp();
//
//    Eigen::MatrixXd integral_exp_A;
//    integral_exp_A = Eigen::MatrixXd::Zero(stateSize, stateSize);
//    const int count_integral_A = 100;
//
//    for (int i = 0; i < count_integral_A; i++) {
//        integral_exp_A += (A * sampling_time * i / count_integral_A).exp()
//                          * sampling_time / count_integral_A;
//    }
//
//    Bd = integral_exp_A * B;
//
//
//    A_.vSetToZero();
//    B_.vSetToZero();
//    C_.vSetIdentity();
//
//    for (int i = 0; i < stateSize; ++i){
//        for (int j=0; j < stateSize; ++j){
//            A_[i][j] = Ad(i,j);
//        }
//    }
//
//    for (int i = 0; i < stateSize; ++i){
//        for (int j=0; j < controlSize; ++j){
//            B_[i][j] = Bd(i,j);
//        }
//    }
//
////    Q.setZero(); R.setZero();
////
////    Q(0,0) = 10; Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10; Q(4,4) = 10;
////    Q(5,5) = 10; Q(6,6) = 10; Q(7,7) = 10; Q(8,9) = 10; Q(9,9) = 10;
////
////    R(0,0) = 10; R(1,1) = 10; R(2,2) = 10;
//
//    eiei.vReInit(A_,B_,C_,10,1);
}

void MulticopterAttitudeControl::runMPClast(Eigen::Vector3f position, Eigen::Vector3f  velocity, Eigen::Vector3f p) {
//
//    MatrixNew::Matrix Setpoint(SS_Z_LEN*MPC_HP_LEN,1);
//
//    for (int i=0; i<MPC_HP_LEN; ++i){
//        Setpoint[i*MPC_HP_LEN+0][0] = 10;
//        Setpoint[i*MPC_HP_LEN+1][0] = 10;
//        Setpoint[i*MPC_HP_LEN+2][0] = -5;
//        Setpoint[i*MPC_HP_LEN+3][0] = 0;
//        Setpoint[i*MPC_HP_LEN+4][0] = 0;
//        Setpoint[i*MPC_HP_LEN+5][0] = 0;
//        Setpoint[i*MPC_HP_LEN+6][0] = 0;
//        Setpoint[i*MPC_HP_LEN+7][0] = 0;
//        Setpoint[i*MPC_HP_LEN+8][0] = 1.0;
//    }
//
//    MatrixNew::Matrix State(SS_X_LEN,1);
//    State[0][0] = position[0];
//    State[1][0] = position[1];
//    State[2][0] = position[2];
//    State[3][0] = velocity[0];
//    State[4][0] = velocity[1];
//    State[5][0] = velocity[2];
//    State[6][0] = p[0];
//    State[7][0] = p[1];
//    State[8][0] = p[2];
//
//    bool up;
//
//    up = eiei.bUpdate(Setpoint, State, u_);

//    cout<<"Update: "<<up<<endl;

}

void
MulticopterAttitudeControl::Run() {
    if (should_exit()) {
        _vehicle_attitude_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // Check if parameters have changed
    if (_parameter_update_sub.updated()) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
        parameters_updated();
    }

    // run controller on attitude updates
     vehicle_attitude_s v_att;

    // Minimal test of NMPC


//    const uint8_t prev_quat_reset_counter = _vehicle_attitude.quat_reset_counter;

    if (_vehicle_attitude_sub.update(&v_att)) {

        // Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
//        const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
        const float dt = 0.004;
        _last_run = v_att.timestamp_sample;

        const Quatf q{v_att.q};

        // Check for new attitude setpoint
		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

			if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
			    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

				_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
				_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
				_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
			}
		}

        // Check for a heading reset
        if (_quat_reset_counter != v_att.quat_reset_counter) {
            const Quatf delta_q_reset(v_att.delta_q_reset);

            // for stabilized attitude generation only extract the heading change from the delta quaternion
            _man_yaw_sp = wrap_pi(_man_yaw_sp + Eulerf(delta_q_reset).psi());

			if (v_att.timestamp > _last_attitude_setpoint) {
				// adapt existing attitude setpoint unless it was generated after the current attitude estimate
				_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
			}

			_quat_reset_counter = v_att.quat_reset_counter;
        }

        /* check for updates in other topics */
        _manual_control_setpoint_sub.update(&_manual_control_setpoint);
        _vehicle_control_mode_sub.update(&_vehicle_control_mode);



//        OmegaSp.setZero();
        // MPC state

//        std::cout << "Ang Vel 1: " << OmegaSp[0] << std::endl;
//        std::cout << "Ang Vel 2: " << OmegaSp[1] << std::endl;
//        std::cout << "Ang Vel 3: " << OmegaSp[2] << std::endl;

        if (_vehicle_land_detected_sub.updated()) {
            vehicle_land_detected_s vehicle_land_detected;

            if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
                _landed = vehicle_land_detected.landed;
            }
        }

        if (_vehicle_status_sub.updated()) {
            vehicle_status_s vehicle_status;

            if (_vehicle_status_sub.copy(&vehicle_status)) {
                _vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
                _vtol = vehicle_status.is_vtol;
                _vtol_in_transition_mode = vehicle_status.in_transition_mode;
                _vtol_tailsitter = vehicle_status.is_vtol_tailsitter;

            }
        }

        bool attitude_setpoint_generated = false;

        const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);

        // vehicle is a tailsitter in transition mode
        const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

        bool run_att_ctrl =
                _vehicle_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);

        if (run_att_ctrl) {

            // Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
            if (_vehicle_control_mode.flag_control_manual_enabled &&
                !_vehicle_control_mode.flag_control_altitude_enabled &&
                !_vehicle_control_mode.flag_control_velocity_enabled &&
                !_vehicle_control_mode.flag_control_position_enabled) {

                generate_attitude_setpoint(q, dt, _reset_yaw_sp);
                attitude_setpoint_generated = true;

            } else {
                _man_x_input_filter.reset(0.f);
                _man_y_input_filter.reset(0.f);
            }

            Vector3f rates_sp = _attitude_control.update(q);
//
//            //Vector3f rates_sp = _att_ctrl;
//
			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now - pid_autotune.timestamp) < 1_s)) {
					rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

            // publish rate setpoint



		    vehicle_rates_setpoint_s rates_setpoint{};

            _vehicle_odometry_sub.update(&_vehicle_odometry);
            _vehicle_positin_setpoint_sub.update(&_vehicle_trajectory_setpoint);

//            uint64_t stop = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

            // Update estimated thrust;
            hIntregationReal(u0_w[0], dt);

            // Get current state (Change from NED to ENU frame) (x->y, y->x, z->-z)
            // Quater from NED to OUR frame is (0,0,1,0)

            // Quaternion transformation from NED to ENU
//            Eigen::Vector4f quat_tran_1 (0.707f,0.707f,0.0,0.0);
//            Eigen::Matrix4f quat_tran_2;
//            quat_tran_2 << 0.0f, 0.0f, -0.707f, 0.707f,
//                    0.0f, 0.0f, 0.707f, 0.707f,
//                    0.707f, -0.707f, 0.0f, 0.0f,
//                    -0.707f, -0.707f, 0.0f, 0.0f;
//                Eigen::Vector4f quaternion (_vehicle_odometry.q[0], _vehicle_odometry.q[1], _vehicle_odometry.q[2], _vehicle_odometry.q[3]);
//                Eigen::Matrix4f quat_matrix;
//                quat_matrix << quaternion[0], -quaternion[3], quaternion[2], -quaternion[1],
//                        quaternion[3], quaternion[0], -quaternion[1], -quaternion[2],
//                        -quaternion[2], quaternion[1], quaternion[0], -quaternion[3],
//                        quaternion[1], quaternion[2], quaternion[3], quaternion[0];
//                Eigen::Vector4f quat_enu;
//                quat_enu = (quat_tran_1.transpose() * quat_matrix * quat_tran_2).transpose();
//
//                Eigen::Vector3f x (_vehicle_odometry.position[1], _vehicle_odometry.position[0], -_vehicle_odometry.position[2]);
//                Eigen::Vector3f x_dot (_vehicle_odometry.velocity[1], _vehicle_odometry.velocity[0], -_vehicle_odometry.velocity[2]);


                Eigen::Vector3f x (_vehicle_odometry.position[0], _vehicle_odometry.position[1], -_vehicle_odometry.position[2]);
                Eigen::Vector3f x_dot (_vehicle_odometry.velocity[0], _vehicle_odometry.velocity[1], -_vehicle_odometry.velocity[2]);
                Eigen::Vector4f quaternion (_vehicle_odometry.q[0], _vehicle_odometry.q[1], _vehicle_odometry.q[2], _vehicle_odometry.q[3]);

                cout << "x: " << x[0] << endl;
                cout << "y: " << x[1] << endl;
                cout << "z: " << x[2] << endl;

                // Calculate coordinates expression of S1 Principle Bundle Map
                Eigen::Vector3f pQ = calculateS1PrincipleBundleMap(quaternion);
                Eigen::Vector3f controlInputW;

                // Set reference here

                // Off board (NED)
            Eigen::Vector3f xd (_vehicle_trajectory_setpoint.position[0],_vehicle_trajectory_setpoint.position[0],
                                -_vehicle_trajectory_setpoint.position[2]);
            Eigen::Vector3f vd (_vehicle_trajectory_setpoint.velocity[0],_vehicle_trajectory_setpoint.velocity[1],
                                -_vehicle_trajectory_setpoint.velocity[2]);

            // ENU
//            Eigen::Vector3f xd (0.0f,0.0f,2.5f);
//            Eigen::Vector3f vd (0.0f,0.0f,0.0f);

            Eigen::Vector4f qd(1.0f,0.0f,0.0f,0.0f); // DEFINE in NED
            Eigen::Vector3f pd;
            pd[0] = 0.71f*f_hover_desired*(2*(qd[1]*qd[3]+qd[0]*qd[2])); pd[1] = 0.71f*f_hover_desired*(2*(qd[2]*qd[3]-qd[0]*qd[1]));
            pd[2] = 0.71f*f_hover_desired*(qd[0]*qd[0]-qd[1]*qd[1]-qd[2]*qd[2]+qd[3]*qd[3]);

//            cout << "pd: " << pd << endl;

                // Base dynamics (Control drone except yaw)
                // MPC controller

//            uint64_t start1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                controlInputW = runAcado(x, x_dot, pQ, xd, vd, pd);
//            uint64_t stop1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//            std::cout << "Cal time: " << stop1 - start1 << std::endl;

                // Paper Controller
//                controlInputW = -30*(x-xd)-28*(x_dot-vd)-9*(pQ-pd);

                // Fiber Dynamics (Control yaw separately) (SUCCESS!!!!)
                const Quatf quat{_vehicle_odometry.q};
                float yaw_ = Eulerf(quat).psi(); // From NED to OURS
//                float yaw_desired = _vehicle_trajectory_setpoint.yaw; // Set desired yaw here IN NED
                float yaw_desired = 3.14/3;
//                yaw_desired = yaw_desired*-1; // CHANGE to ENU
                float w0 = -2.0f*(yaw_-yaw_desired);

                // Combine base and fiber
                u0_w = getFeedbackDynamicExtension(pQ, controlInputW, w0);
                hIntgration(u0_w[0], dt); // h_desired
                OmegaSp = bodyAngVelSp(u0_w, quaternion);

                // Use default set point
                rates_setpoint.roll = rates_sp(0);
			    rates_setpoint.pitch = rates_sp(1);
			    rates_setpoint.yaw = rates_sp(2);

                // Check default set point
//                cout << "Rate Real 1: " << rates_sp(0) << endl;
//                cout << "Rate Real 2: " << rates_sp(1) << endl;
//                cout << "Rate Real 3: " << rates_sp(2) << endl;

                // Set thrust set point (How??????) (Possibly the integration of h)
//                _thrust_setpoint_body(2) = -(f_desired/f_max_desired);
//            _thrust_setpoint_body(2) = -(1-(xd-x).norm())*0.7f;
//                    _thrust_setpoint_body(2) = -(0.095f+0.048f*f_from_h-0.0006f*f_from_h*f_from_h);
//                _thrust_setpoint_body(2) = -(f_from_h/30.0f+0.5f);

//            cout << "Thrust desired true: " << _thrust_setpoint_body(2) << endl;
//            cout << "Thrust desired ours: " << f_desired/f_max_desired << endl;

                _thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			    rates_setpoint.timestamp = hrt_absolute_time();

                // Use set point from purposed controller (Set points are calculated in OUR frame, changes to NED)

//                    rates_setpoint.roll = OmegaSp[1];
//                    rates_setpoint.pitch = OmegaSp[0];
//                    rates_setpoint.yaw = -OmegaSp[2];

                    rates_setpoint.roll = OmegaSp[0];
                    rates_setpoint.pitch = OmegaSp[1];
                    rates_setpoint.yaw = OmegaSp[2];


                // Check set point
//                cout << "Omega1 controller: " << OmegaSp[0] << endl;
//                cout << "Omega2 controller: " << OmegaSp[1] << endl;
//                cout << "Omega3 controller: " << OmegaSp[2] << endl;


                // Publish to rate controller (internal)
			_vehicle_rates_setpoint_pub.publish(rates_setpoint);

                // Publish to ROS2 node (off-board)
            _vehicle_rates_setpoint_send_pub.publish(rates_setpoint);


            // reset yaw setpoint during transitions, tailsitter.cpp generates
            // attitude setpoint for the transition
            _reset_yaw_sp = !attitude_setpoint_generated || _landed || (_vtol && _vtol_in_transition_mode);


//            std::cout << "Latency1: " << stop - start << std::endl;

//            start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }
        perf_end(_loop_perf);
    }

}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
