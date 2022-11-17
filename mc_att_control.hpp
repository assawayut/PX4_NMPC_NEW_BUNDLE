/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint_send.h>
#include <uORB/topics/vehicle_torque_setpoint_custom.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <math.h>
#include <vector>
#include <AttitudeControl.hpp>
#include <lapacke.h>
#include "mpc.h"


extern "C"{
    #include "acado_common.h"
    #include "acado_auxiliary_functions.h"
//    #include "solver.h"
}

#include "acado_qpoases_interface.hpp"

#define NX          ACADO_NX //num states
#define NXA         ACADO_NXA //num algebraic variables
#define NU          ACADO_NU // num control inputs
#define NP          ACADO_NP // num parameters
#define NY          ACADO_NY // num of reference on 0->N-1
#define NYN         ACADO_NYN // num of reference on N
#define NH           ACADO_N // num horizon
#define NUM_STEPS   10 // Real-time iterations
#define VERBOSE     0 // Silent iteration

    ACADOvariables  acadoVariables;
    ACADOworkspace  acadoWorkspace;
//    Vars vars;
//    Params params;
//    Workspace work;
//    Settings settings;

lapack_logical select_lhp(const double *real, const double *imag)
{
    return *real < 0.0;
}

using namespace time_literals;
using namespace std;

extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterAttitudeControl(bool vtol = false);
	~MulticopterAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();



//    int iter;
//    acado_timer t;

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	float throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt, bool reset_yaw_sp);

    /* Attitude controller*/

    void control_attitude();
    void publish_rates_setpoint();
    //Eigen::Matrix<float, 3, 1> control_attitude_rates(float dt, float roll_rate, float pitch_rate, float yaw_rate, const matrix::Quatf &q);

    void control_attitude_rates(float dt);

//    void initialization ();
//    void runMPC ();

    Eigen::Vector4f getFeedbackDynamicExtension(Eigen::Vector3f p, Eigen::Vector3f w, float w_0);
    void hIntgration(float u0, float dt);
    void hIntregationReal(float u0, float dt);
    Eigen::Vector3f bodyAngVelSp(Eigen::Vector4f dynExt, Eigen::Vector4f quat);
    Eigen::Vector3f calculateS1PrincipleBundleMap(Eigen::Vector4f quat);

    void initAcado();
    Eigen::Vector3f runAcado(Eigen::Vector3f position, Eigen::Vector3f  velocity, Eigen::Vector3f p,
                             Eigen::Vector3f xd_, Eigen::Vector3f vd_, Eigen::Vector3f pd_);

    void mpcLast();
    void runMPClast(Eigen::Vector3f position, Eigen::Vector3f  velocity, Eigen::Vector3f p);

    Eigen::MatrixXd solveCare(Eigen::MatrixXd Q, Eigen::MatrixXd R);

    void loadData();
    void setGain(Eigen::Vector3f p);


	AttitudeControl _attitude_control; /**< class for attitude control calculations */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};       /**< manual control setpoint subscription */
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};         /** < Vehicle rate setpoint subscription > */
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};   /**< vehicle attitude setpoint subscription */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};             /**< vehicle control mode subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};                         /**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};           /**< vehicle land detected subscription */
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
    uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
    uORB::Subscription _vehicle_positin_setpoint_sub{ORB_ID(trajectory_setpoint)};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};    /**< rate setpoint publication */
	uORB::Publication<vehicle_attitude_setpoint_s>  _vehicle_attitude_setpoint_pub;
    uORB::Publication<vehicle_rates_setpoint_s>  _vehicle_rates_setpoint_send_pub{ORB_ID(vehicle_rates_setpoint_send)};

	manual_control_setpoint_s       _manual_control_setpoint {};    /**< manual control setpoint */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< vehicle control mode */
    trajectory_setpoint_s _vehicle_trajectory_setpoint {};

    vehicle_attitude_setpoint_s _vehicle_attitude_setpoint {}; /** < vehicle attitude setpoint > */
    vehicle_attitude_s _vehicle_attitude {}; /** < vehicle attitude > */
    vehicle_angular_acceleration_s _vehicle_ang_acc {};
    vehicle_angular_velocity_s _vehicle_ang_vel {};
    vehicle_rates_setpoint_s _vehicle_rate_setpoint {};
    vehicle_torque_setpoint_s _vehicle_torque_setpoint {};
    vehicle_odometry_s _vehicle_odometry {};

	perf_counter_t  _loop_perf;             /**< loop duration performance counter */

    matrix::Vector3f _rate_setpoint; /**< angular rate setpoint >*/
	matrix::Vector3f _thrust_setpoint_body; /**< body frame 3D thrust vector */

	float _man_yaw_sp{0.f};                 /**< current yaw setpoint in manual mode */
	float _man_tilt_max;                    /**< maximum tilt allowed for manual flight [rad] */

	AlphaFilter<float> _man_x_input_filter;
	AlphaFilter<float> _man_y_input_filter;

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};

	bool _landed{true};
	bool _reset_yaw_sp{true};
	bool _vehicle_type_rotary_wing{true};
	bool _vtol{false};
	bool _vtol_tailsitter{false};
	bool _vtol_in_transition_mode{false};

	uint8_t _quat_reset_counter{0};
    uint64_t start = 0;

//    Eigen::Matrix<float, 6, 1> QPhild(const Eigen::Matrix<float, 6, 6> &E,
//                                      Eigen::Matrix<float, 6, 1> &F, const Eigen::Matrix<float, 24, 6> &CC,
//                                      Eigen::Matrix<float, 24, 1> &d);
//    Eigen::Matrix<float, 6, 1> x_state;                // State vector
//    Eigen::Matrix<float, 6, 1> x_prev;                 // Previous State vector
//    Eigen::Matrix<float, 3, 1> des;                    // Reference rates vector
//    Eigen::Matrix<float, 3, 1> u;                      // Input vector
//    Eigen::Matrix<float, 9, 1> Xf;                     // Augmented state vector
//    Eigen::Matrix<float, 3, 1> y_output;                      // Output vector
//    Eigen::Matrix<float, 3, 1> x_curr;                 // Current state vector
//    Eigen::Matrix<float, 15, 9> P;                     // Output prediction matrix, P
//    Eigen::Matrix<float, 15, 6> H;                     // Output prediction matrix, H
//    Eigen::Matrix<float, 6, 15> H_trans;               // Transpose of H matrix
//    Eigen::Matrix<float, 3, 1> dist;                   // Disturbance matrix
//    Eigen::Matrix<float, 6, 6> E;                      // QP matrix
//    Eigen::Matrix<float, 6, 6> W;                      // Input weight
//    Eigen::Matrix<float, 24, 6> CC;                    // Constraint matrix
//    Eigen::Matrix<float, 24, 1> dd;                    // Constraint vector
//    Eigen::Matrix<float, 24, 3> dupast;                // Past constraint matrix

//    Vars vars;
//    Params params;
//    Workspace workspace;
//    Settings settings;

    const int stateSize = 9;
    const int controlSize = 3;
    float massQuad = 1.5;
    float g = 9.81;
    int sampling_time = 0.01;

    Eigen::Matrix<double, 9, 9> Ad;   //dynamics matrix
    Eigen::Matrix<double, 9, 3> Bd;   //transfer matrix
    Eigen::Matrix<double, 9, 3> Cd;

//    Eigen::Vector3f controlInputW;


    float u_old;
    float h = 1;
    const float h_max = 5.57;
    const float h_min = 1;


    // Desired
    float f_hover_desired = 14.715; //mg
    float f_max_desired = 14.715/0.72;
    float f_init_desired = f_max_desired*0.08f;
    float f_desired = f_init_desired;
    float h_init_desired = sqrt(f_init_desired);
    float h_old_desired = h_init_desired;
    float h_desired = h_init_desired;

    // Real
    float h_old_real = h_init_desired;
    float h_real = h_init_desired;
    float f_real = f_init_desired;

    Eigen::Vector4f u0_w;


    Eigen::Vector3f OmegaSp;
//
//    MatrixNew::Matrix A_ = MatrixNew::Matrix(SS_X_LEN, SS_X_LEN);
//    MatrixNew::Matrix B_ = MatrixNew::Matrix(SS_X_LEN, SS_U_LEN);
//    MatrixNew::Matrix C_ = MatrixNew::Matrix(SS_Z_LEN, SS_X_LEN);
//    MatrixNew::Matrix SP_ = MatrixNew::Matrix((MPC_HP_LEN*SS_Z_LEN), 1);
//    MatrixNew::Matrix x_ = MatrixNew::Matrix(SS_X_LEN, 1);
//    MatrixNew::Matrix u_ = MatrixNew::Matrix(SS_U_LEN, 1);
//    MatrixNew::Matrix z_ = MatrixNew::Matrix(SS_Z_LEN, 1);
//
//    MPC eiei;

    DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,

		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,    /**< maximum tilt allowed for manual flight */
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,       /**< scaling factor from stick to yaw rate */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>)   _param_mpc_manthr_min,      /**< minimum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,         /**< maximum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,       /**< throttle at stationary hover */
		(ParamInt<px4::params::MPC_THR_CURVE>)      _param_mpc_thr_curve        /**< throttle curve behavior */
	)
};

