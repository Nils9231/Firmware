/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file uuv_att_control_main.cpp
 * UUV attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * based on mc_att_control_main.cpp from
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * adjusted by
 * @author Nils Rottmann    <Nils.Rottmann@tuhh.de> (-12/2017)
 * @author Daniel Duecker   <Daniel.Duecker@tuhh.de> (07/2018-)
 *
 * The controller has two loops: P loop for position and angular error and PD loop for velocity and angular rate error.
 */


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
// uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>               // orientation data
#include <uORB/topics/vehicle_local_position.h>         // position data
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

// system libraries
#include <parameters/param.h>
#include <systemlib/err.h>
//#include <systemlib/circuit_breaker.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
// internal libraries
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>
//#include <lib/tailsitter_recovery/tailsitter_recovery.h>

using namespace matrix;

// UUV Attitude Controller
extern "C" __EXPORT int uuv_att_control_main(int argc, char *argv[]);

// the class from which an instance will be initiated by starting this application
class UUVAttControl
{
public:
	// Constructor
	UUVAttControl();

	// Destructor, also kills the main task
	~UUVAttControl();

	// Start the multicopter attitude control task, @return OK on success.
	int	start();

private:

	bool	_task_should_exit;		// if true, task_main() should exit
	int		_control_task;			// task handle

	// topic subscriptions
	int		_v_att_sub;		        // orientation data
	int     _v_pos_sub;             // position data
	int		_params_sub;			// parameter updates subscription
	int		_v_traj_sp_sub;			// trajectory setpoint subscription

	// topic publications
	orb_advert_t	_actuators_0_pub;		    // attitude actuator controls publication
	orb_id_t        _actuators_id;	            // pointer to correct actuator controls0 uORB metadata structure

	// topic structures, in this structures the data of the topics are stored
	struct actuator_controls_s			_actuators;			    // actuator controls
	struct vehicle_attitude_s		    _v_att;		            // attitude data
    struct trajectory_setpoint_s	    _v_traj_sp;			    // trajectory setpoint

	// performance counters
	perf_counter_t	_loop_perf;
	perf_counter_t	_controller_latency_perf;

	// time counter
	float t_ges;

    Matrix<float, 3, 3>  _I;				// identity matrix

	struct {
		param_t K_RX;
		param_t K_RY;
		param_t K_RZ;
		param_t K_WX;
		param_t K_WY;
		param_t K_WZ;
		param_t K_F;
		param_t K_M;
		param_t L;
		param_t ROLL;
		param_t PITCH;
		param_t YAW;
	}		_params_handles;		// handles for to find parameters

	struct {
		// gain matrices
        Matrix<float, 3, 3> K_r;         // orientation
        Matrix<float, 3, 3> K_w;         // angular velocity
		// Force and Moment scaling factors
		float k_F;
		float k_M;
		// Lifting arm
		float L;
		float roll;
		float pitch;
		float yaw;
	}		_params;

	// get orientation error
    Vector3f rotError(Matrix<float, 3, 3> R, Matrix<float, 3, 3> R_des);

	// attitude controller.
	void		att_control(float dt);

	// Update our local parameter cache.
	int			parameters_update();                // checks if parameters have changed and updates them

	// Check for parameter update and handle it.
	void		parameter_update_poll();            // receives parameters

    // update actual trajectory setpoint
	void        trajectory_setpoint_poll();

	// Shim for calling task_main from task_create.
	static void	task_main_trampoline(int argc, char *argv[]);

	// Main attitude control task.
	void		task_main();
};


namespace uuv_att_control
{
UUVAttControl	*g_control;
}

// constructor of class HippocampusPathControl
UUVAttControl::UUVAttControl() :

	// First part is about function which are called with the constructor
	_task_should_exit(false),
	_control_task(-1),

	// subscriptions
	_v_att_sub(-1),
	_params_sub(-1),
	_v_traj_sp_sub(-1),

	// publications
	_actuators_0_pub(nullptr),
	_actuators_id(nullptr),

	// performance counters
	_loop_perf(perf_alloc(PC_ELAPSED, "uuv_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

// here starts the allocation of values to the variables
{
	// define publication settings
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_v_traj_sp, 0, sizeof(_v_traj_sp));

	// set parameters to zero
	_params.K_r.zero();
	_params.K_w.zero();
	_params.L = 0.0f;
	_params.roll = 0.0f;
	_params.pitch = 0.0f;
	_params.yaw = 0.0f;

	// Set time to zero
	t_ges = 0.0f;

	// allocate Identity matrix
	_I.identity();

	// allocate parameter handles
        _params_handles.K_RX		    = 	param_find("UUV_ATT_K_RX");
        _params_handles.K_RY		    = 	param_find("UUV_ATT_K_RY");
        _params_handles.K_RZ		    = 	param_find("UUV_ATT_K_RZ");
        _params_handles.K_WX		    = 	param_find("UUV_ATT_K_WX");
        _params_handles.K_WY		    = 	param_find("UUV_ATT_K_WY");
        _params_handles.K_WZ		    = 	param_find("UUV_ATT_K_WZ");
        _params_handles.K_F		        = 	param_find("UUV_ATT_K_F");
        _params_handles.K_M		        = 	param_find("UUV_ATT_K_M");
        _params_handles.L	            = 	param_find("UUV_ATT_L");
        _params_handles.ROLL	        = 	param_find("UUV_ATT_ROLL");
        _params_handles.PITCH	        = 	param_find("UUV_ATT_PITCH");
        _params_handles.YAW	            = 	param_find("UUV_ATT_YAW");

	// fetch initial parameter values
	parameters_update();
}

// destructor of class HippocampusPathControl
UUVAttControl::~UUVAttControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	uuv_att_control::g_control = nullptr;
}

// updates parameters
int UUVAttControl::parameters_update()
{
	float v;

	param_get(_params_handles.K_RX, &v);
	_params.K_r(0,0) = v;
	param_get(_params_handles.K_RY, &v);
	_params.K_r(1,1) = v;
	param_get(_params_handles.K_RZ, &v);
	_params.K_r(2,2) = v;
	param_get(_params_handles.K_WX, &v);
	_params.K_w(0,0) = v;
	param_get(_params_handles.K_WY, &v);
	_params.K_w(1,1) = v;
	param_get(_params_handles.K_WZ, &v);
	_params.K_w(2,2) = v;
	param_get(_params_handles.K_F, &v);
	_params.k_F = v;
	param_get(_params_handles.K_M, &v);
	_params.k_M = v;
	param_get(_params_handles.L, &v);
	_params.L = v;
	param_get(_params_handles.ROLL, &v);
	_params.roll = v;
	param_get(_params_handles.PITCH, &v);
	_params.pitch = v;
	param_get(_params_handles.YAW, &v);
	_params.yaw = v;

	return OK;
}

// check for parameter updates
void UUVAttControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

// Get the Setpoint from the trajectory planner
void UUVAttControl::trajectory_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_traj_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(trajectory_setpoint), _v_traj_sp_sub, &_v_traj_sp);
	}
}

// Gives back orientation error between R and R_des
Vector3f UUVAttControl::rotError(Matrix<float, 3, 3> R, Matrix<float, 3, 3> R_des)
{
    // extract orientation vectors
    Vector3f e1(1,0,0);
    Vector3f e3(0,0,1);
    Vector3f x_B = R * e1; //.get_colValues(0);
    Vector3f x_B_des = R_des * e1; //.get_colValues(0);
    Vector3f z_B_des = R_des * e3; //.get_colValues(2);

    // extracting one rotation from the rotation matrix, necessary due to singularities in the (R_des^T * R - R^T * R_des) approach
	// rotation axis for rotation between x_B and x_B_des, in B coordinates (not normalized yet)
    Vector3f e_r = R.transpose() * (x_B_des % x_B);

	// calculate the angle errors using norm of cross product and dot product
	float x_B_sin = e_r.length();
	float x_B_cos = x_B * x_B_des;

	// rotation matrix after pitch/yaw only rotation, thus between R and R_py are only a roll rotation left
    Matrix<float, 3, 3> R_py;

	// check if x_B and x_B_des are non parallel, otherwise we would not have rotations pitch and yaw
	if (x_B_sin > 0.0f) {
		// calculate axis angle representation of the pitch/yaw rotation
		float e_R_angle = atan2f(x_B_sin, x_B_cos);
        Vector3f e_R_axis = e_r / x_B_sin;           // normalize axis

		// get the error vector of the rotation in B coordinates
		e_r = e_R_axis * e_R_angle;

		// get the cross product matrix for e_R_axis to calculate the Rodrigues formula
        Matrix<float, 3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_axis(2);
		e_R_cp(0, 2) = e_R_axis(1);
		e_R_cp(1, 0) = e_R_axis(2);
		e_R_cp(1, 2) = -e_R_axis(0);
		e_R_cp(2, 0) = -e_R_axis(1);
		e_R_cp(2, 1) = e_R_axis(0);

		// rotation matrix after pitch/yaw only rotation, thus between R and R_py are only a roll rotation left, in World coordinates
		R_py = R * (_I + e_R_cp * x_B_sin + e_R_cp * e_R_cp * (1.0f - x_B_cos));

	} else {
		// zero pitch/yaw rotation
		R_py = R;
	}

	//R_py and R_des have the same X axis, calculate roll error
    Vector3f z_B_py(R_py(0, 2), R_py(1, 2), R_py(2, 2));
	e_r(0) = atan2f((z_B_des % z_B_py) * x_B_des, z_B_py * z_B_des);

	return e_r;
}


/**
 * uuv attitude controller
 * Input: 'desired orientation'
 * Output: 'actuators_control'
 */
void UUVAttControl::att_control(float dt)
{
     // actualize setpoint data
	//trajectory_setpoint_poll();

    // Count time
    t_ges = t_ges + dt;

	//*******************************
	//  Declaration of Variables
	//*******************************
	// define error vectors
    Vector3f e_r;            // orientation error
//	Vector3f e_p;            // position error
//	Vector3f e_v;            // velocity error
//	Matrix<float, 3, 3> e_r_matrix;  // orientation error matrix
//	Vector3f e_w;            // angular velocity error

    float u_1;                      // thrust input
    Vector3f u_24;           // desired actuator signals

/*
	// get actual position data
    Vector3f r;                      // actual position
    Vector3f rd;                     // actual velocity
    Vector3f rdd;                    // actual acceleration
    Vector3f rddd;
*/

	// rotation matrices and angular velocity vectors
    Vector3f e_w(_v_att.rollspeed, _v_att.pitchspeed, _v_att.yawspeed);      // angular velocity error
    Matrix<float, 3, 3> R;                                                   // actual rotation matrix
    Matrix<float, 3, 3> R_des;                                               // desired rotation matrix


	// get current rotation matrix from control state quaternions, the quaternions are generated by the
	// attitude_estimator_q application using the sensor data
    matrix::Quatf q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	// create rotation matrix for the quaternion when post multiplying with a column vector x'=R*x
	R = q_att.to_dcm();

	// orientation vectors
    Vector3f x_B(R(0, 0), R(1, 0), R(2,0));  // orientation body x-axis (in world coordinates)
    Vector3f y_B(R(0, 1), R(1, 1), R(2,1));  // orientation body y-axis (in world coordinates)
    Vector3f z_B(R(0, 2), R(1, 2), R(2,2));  // orientation body z-axis (in world coordinates)
    Vector3f x_B_des;                        // orientation body x-axis desired
    Vector3f y_B_des;                        // orientation body y-axis desired
    Vector3f z_B_des;                        // orientation body z-axis desired

    // Calculate Desired Rotation Matrix
	float c_roll = cosf(_params.roll);
	float s_roll = sinf(_params.roll);
	float c_pitch = cosf(_params.pitch);
	float s_pitch = sinf(_params.pitch);
	float c_yaw = cosf(_params.yaw);
	float s_yaw = sinf(_params.yaw);

	R_des(0,0) = c_pitch*c_yaw;
	R_des(0,1) = s_roll*s_pitch*c_yaw - c_roll*s_yaw;
	R_des(0,2) = c_roll*s_pitch*c_yaw + s_roll*s_yaw;
	R_des(1,0) = c_pitch*s_yaw;
	R_des(1,1) = s_roll*s_pitch*s_yaw + c_roll*c_yaw;
	R_des(1,2) = c_roll*s_pitch*s_yaw - s_roll*c_yaw;
	R_des(2,0) = -s_pitch;
	R_des(2,1) = s_roll*c_pitch;
	R_des(2,2) = c_roll*c_pitch;

    // Get the rotation error
	e_r = rotError(R, R_des);
    Vector3f e1(1,0,0);
    Vector3f e2(0,1,0);
    Vector3f e3(0,0,1);
    x_B_des = R_des * e1; //.get_colValues(0);
    y_B_des = R_des * e2; //.get_colValues(1);
    z_B_des = R_des * e3; //.get_colValues(2);

    // Set Thrust to zero
	u_1 = 0.0f;
    //u_1 = F_des*x_b

	// calculate input over feedback loop
	u_24 = -_params.K_r * e_r - _params.K_w * e_w;

	// scale roll
	u_24(0) = u_24(0) * (_params.L * _params.k_F / _params.k_M);

	// give the inputs to the actuators
	_actuators.control[0] = u_24(0);            // roll
	_actuators.control[1] = u_24(1);            // pitch
    _actuators.control[2] = u_24(2);           // yaw
	_actuators.control[3] = u_1;                // thrust
}

// Just starts the task_main function
void UUVAttControl::task_main_trampoline(int argc, char *argv[])
{
	uuv_att_control::g_control->task_main();
}

// this is the main_task function which does the control task
void UUVAttControl::task_main()
{

	PX4_INFO("uuv_att_control has been started!");
	// subscribe to uORB topics
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	// initialize parameters cache
	parameters_update();

	// wakeup source: vehicle pose
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		// wait for up to 100ms for data, we try to poll the data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out - periodic check for _task_should_exit
		if (pret == 0) {
			PX4_INFO("Got no data in 100ms!");
			continue;
		}

		// this is undesirable but not much we can do - might want to flag unhappy status
		if (pret < 0) {
			warn("uuv_att_control: poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		// run controller on pose changes
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;   // calculate the time delta_t between two runs
			last_run = hrt_absolute_time();

			// guard against too small (< 2ms) and too large (> 20ms) dt's
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			// copy orientation data
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			// do path control
			att_control(dt);

			// publish actuator timestamps
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _v_att.timestamp;

			if (_actuators_0_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
				perf_end(_controller_latency_perf);

			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}


			_actuators_id = ORB_ID(actuator_controls_0);

			// check for parameter updates
			parameter_update_poll();
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

// start function
int UUVAttControl::start()
{
	ASSERT(_control_task == -1);        // give a value -1

	// start the control task, performs any specific accounting, scheduler setup, etc.
	_control_task = px4_task_spawn_cmd("uuv_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5000, //
					   (px4_main_t)&UUVAttControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

// main function
int uuv_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: uuv_att_control {start|stop|status}");
		return 1;
	}

	// if command is start, then first control if class exists already, if not, allocate new one
	if (!strcmp(argv[1], "start")) {

		if (uuv_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		// allocate new class HippocampusPathControl
		uuv_att_control::g_control = new UUVAttControl();

		// check if class has been allocated, if not, give back failure
		if (uuv_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		// if function start() can not be called, delete instance of HippocampusPathControl and allocate null pointer
		if (OK != uuv_att_control::g_control->start()) {
			delete uuv_att_control::g_control;
			uuv_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	// if command is start, check if class exists, if not can not stop anything
	if (!strcmp(argv[1], "stop")) {
		if (uuv_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		// if class exists, delete it and allocate null pointer
		delete uuv_att_control::g_control;
		uuv_att_control::g_control = nullptr;
		return 0;
	}

	// if command is status and class exists, give back running, else give back not running
	if (!strcmp(argv[1], "status")) {
		if (uuv_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
