/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file uuv_leafo_app.cpp
 *
 *
 * @author Nils Timmermann <Nils.Timmermann@tuhh.de>
 *
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

// drivers
#include <drivers/drv_hrt.h>

// system libraries
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <systemlib/mavlink_log.h>

// internal libraries
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>

// Include uORB and the required topics for this app
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>                // this topics hold the acceleration data
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>               // this topic holds the orientation of the hippocampus
#include <uORB/topics/vehicle_local_position.h>         // this topic holds all position and speed information
#include <uORB/topics/uuv_one_pose.h>                   // holds poition Info for first boat
#include <uORB/topics/uuv_two_pose.h>                   // holds poition Info for second boat
#include <uORB/topics/uuv_three_pose.h>                 // holds poition Info for third boat
#include <uORB/topics/uuv_four_pose.h>                  // holds poition Info for fourth boat
#include <uORB/topics/uuv_five_pose.h>                  // holds poition Info for fifth boat
#include <uORB/topics/parameter_update.h>


extern "C" __EXPORT int uuv_leafo_app_main(int argc, char *argv[]);

class UUVLeaFo
{
public:
    UUVLeaFo();
    ~UUVLeaFo();
     int start();
private:
     bool		_task_should_exit;		/**< if true, task should exit */
     int		_main_task;			/**< handle for task */
     orb_advert_t	_mavlink_log_pub;
     orb_advert_t	_actuator_pub;
     orb_advert_t       _pose_pub;
     int                _sensor_sub;
     int                _params_sub;
     int                _vehicle_attitude_sub;
     int                _vehicle_local_position_sub;
     int                _uuv_one_pose_sub;
     int                _uuv_two_pose_sub;
     int                _uuv_three_pose_sub;
     int                _uuv_four_pose_sub;
     int                _uuv_five_pose_sub;
     struct actuator_controls_s _actuators;


     struct{
         param_t NUM;               // Number of Vehicles
         param_t Order;             // Ordering Number of Vehicle
         param_t IfChain;           // Chain or all follow first
         param_t shouldRun;         // Start While
         param_t Ksp;               // Speed constand
         param_t Kpy;               // proportional Gain Yaw_rate
         param_t Kiy;               // integrator Gain Yaw_rate
         param_t Kdy;               // differentiator Gain Yaw_rate
         param_t Kt;                // Trajectory drirection point
         param_t T_x;               // desired Trajectory
         param_t T_y;
         param_t T_z;
         param_t Toff_x;            // desired Trajectory offset
         param_t Toff_y;
         param_t Toff_z;
     } _params_handles;

     struct{
         int NUM;
         int Order;
         int IfChain;
         int shouldRun;
         double Ksp;                // speed constant
         double Kpy;                // proportional Gain Yaw_rate
         double Kiy;                // integrator Gain Yaw_rate
         double Kdy;                // differentiator Gain Yaw_rate
         double Kt;
         double T_x;                // desired Trajectory
         double T_y;
         double T_z;
         double Toff_x;             // desired Trajectory offset
         double Toff_y;
         double Toff_z;
     } _params;

     int		actuators_publish();

     int                parameters_update();

     void               parameter_update_poll();

     void               task_main();

     int                actuator_publish();

     static int	task_main_trampoline(int argc, char *argv[]);


};


namespace leafo_app
{
UUVLeaFo	*g_leafo_app;
}

UUVLeaFo::UUVLeaFo():

    _task_should_exit(false),
    _main_task(-1),
    _mavlink_log_pub(nullptr),
    _actuator_pub(nullptr),
    _params_sub(-1),
    _actuators {}
{
    // define publication settings
    memset(&_actuators, 0, sizeof(_actuators));

        // allocate parameter handles
    _params_handles.NUM         = param_find("UUV_LEAFO_NUM");       // Number of Vehicles in Cicle
    _params_handles.Order       = param_find("UUV_LEAFO_ORDER");     // Ordering Number of Vehicle
    _params_handles.IfChain     = param_find("UUV_LEAFO_IC");
    _params_handles.shouldRun   = param_find("UUV_LEAFO_SR");
    _params_handles.Ksp         = param_find("UUV_LEAFO_KSP");       // speed constant
    _params_handles.Kpy         = param_find("UUV_LEAFO_KYP");       // proportional Gain Yaw_rate
    _params_handles.Kiy         = param_find("UUV_LEAFO_KYI");       // integrator Gain Yaw_rate
    _params_handles.Kdy         = param_find("UUV_LEAFO_KYD");       // differentiator Gain Yaw_rate
    _params_handles.Kt          = param_find("UUV_LEAFO_KT");
    _params_handles.T_x         = param_find("UUV_LEAFO_T_X");       // desired Trqajectory
    _params_handles.T_y         = param_find("UUV_LEAFO_T_Y");
    _params_handles.T_z         = param_find("UUV_LEAFO_T_Z");
    _params_handles.Toff_x      = param_find("UUV_LEAFO_TOFF_X");    // desired Trajectory offset
    _params_handles.Toff_y      = param_find("UUV_LEAFO_TOFF_Y");
    _params_handles.Toff_z      = param_find("UUV_LEAFO_TOFF_Z");



    // fetch initial parameter values
    parameters_update();
}


UUVLeaFo::~UUVLeaFo()
{
      if (_main_task != -1) {

              /* task wakes up every 100ms or so at the longest */
              _task_should_exit = true;

              /* wait for a second for the task to quit at our request */
              unsigned i = 0;

              do {
                      /* wait 20ms */
                      usleep(20000);

                      /* if we have given up, kill it */
                      if (++i > 50) {
                              px4_task_delete(_main_task);
                              break;
                      }
              } while (_main_task != -1);
      }

      leafo_app::g_leafo_app = nullptr;
}

int UUVLeaFo::parameters_update()
{
        float v;

        param_get(_params_handles.NUM, &v);
        _params.NUM = v;
        param_get(_params_handles.Order, &v);
        _params.Order = v;
        param_get(_params_handles.IfChain, &v);
        _params.IfChain = v;
        param_get(_params_handles.shouldRun, &v);
        _params.shouldRun = v;
        param_get(_params_handles.Ksp, &v);
        _params.Ksp = v;
        param_get(_params_handles.Kpy, &v);
        _params.Kpy = v;
        param_get(_params_handles.Kiy, &v);
        _params.Kiy = v;
        param_get(_params_handles.Kdy, &v);
        _params.Kdy = v;
        param_get(_params_handles.Kt, &v);
        _params.Kt = v;
        param_get(_params_handles.T_x, &v);
        _params.T_x = v;
        param_get(_params_handles.T_y, &v);
        _params.T_y = v;
        param_get(_params_handles.T_z, &v);
        _params.T_z = v;
        param_get(_params_handles.Toff_x, &v);
        _params.Toff_x = v;
        param_get(_params_handles.Toff_y, &v);
        _params.Toff_y = v;
        param_get(_params_handles.Toff_z, &v);
        _params.Toff_z = v;



        return OK;
}

void UUVLeaFo::parameter_update_poll()
{
    bool updated;

    /* Check if parameters have changed */
    orb_check(_params_sub, &updated);

    if (updated){
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
        parameters_update();
    }
}

int
UUVLeaFo::start()
{
    ASSERT(_main_task == -1);

    /* start the task */
    _main_task = px4_task_spawn_cmd("leafo_app",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT + 15,
                                    1500,
                                    (px4_main_t)&UUVLeaFo::task_main_trampoline,
                                    nullptr);

    if (_main_task < 0) {
            warn("task start failed");
            return -errno;
    }

    return OK;
}

int
UUVLeaFo::actuators_publish()
{
        _actuators.timestamp = hrt_absolute_time();

        // lazily publish _actuators only once available
        if (_actuator_pub != nullptr) {
                PX4_INFO("ACT:\t%8.4f\t%8.4f \n",
                        (double)_actuators.control[2],
                        (double)_actuators.control[3]);
                return orb_publish(ORB_ID(actuator_controls_0), _actuator_pub, &_actuators);


        } else {
                _actuator_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

                if (_actuator_pub != nullptr) {
                        return OK;

                } else {
                        return -1;
                }
        }
}



void
UUVLeaFo::task_main()
{

        mavlink_log_info(&_mavlink_log_pub, "[leafo_app] has been started!");

       // PX4_INFO("auv_hippocampus_leafo_app has been started!");

        /* subscribe to sensor_combined topic */
        _sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
        /* subscribe to control_state topic */
        _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        /* subscribe to localization topic */
        _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
        /* subscribe to first uuv position topic */
        _uuv_one_pose_sub = orb_subscribe(ORB_ID(uuv_one_pose));
        /* subscribe to second uuv position topic */
        _uuv_two_pose_sub = orb_subscribe(ORB_ID(uuv_two_pose));
        /* subscribe to third uuv position topic */
        _uuv_three_pose_sub = orb_subscribe(ORB_ID(uuv_three_pose));
        /* subscribe to fourth uuv position topic */
        _uuv_four_pose_sub = orb_subscribe(ORB_ID(uuv_four_pose));
        /* subscribe to fifth uuv position topic */
        _uuv_five_pose_sub = orb_subscribe(ORB_ID(uuv_five_pose));


        // wakeup source(s)
        int s=3+_params.NUM;
        px4_pollfd_struct_t fds[s];

        // Setup of loop
        switch(_params.NUM){
            case 0:
                fds[0].fd = _sensor_sub;
                fds[0].events = POLLIN;
                fds[1].fd = _vehicle_attitude_sub;
                fds[1].events = POLLIN;
                fds[2].fd = _vehicle_local_position_sub;
                fds[2].events = POLLIN;
                break;
            case 1:
                fds[0].fd = _sensor_sub;
                fds[0].events = POLLIN;
                fds[1].fd = _vehicle_attitude_sub;
                fds[1].events = POLLIN;
                fds[2].fd = _vehicle_local_position_sub;
                fds[2].events = POLLIN;
                fds[3].fd = _uuv_one_pose_sub;
                fds[3].events = POLLIN;
                break;
            case 2:
                fds[0].fd = _sensor_sub;
                fds[0].events = POLLIN;
                fds[1].fd = _vehicle_attitude_sub;
                fds[1].events = POLLIN;
                fds[2].fd = _vehicle_local_position_sub;
                fds[2].events = POLLIN;
                fds[3].fd = _uuv_one_pose_sub;
                fds[3].events = POLLIN;
                fds[4].fd = _uuv_two_pose_sub;
                fds[4].events = POLLIN;
                break;
            case 3:
                fds[0].fd = _sensor_sub;
                fds[0].events = POLLIN;
                fds[1].fd = _vehicle_attitude_sub;
                fds[1].events = POLLIN;
                fds[2].fd = _vehicle_local_position_sub;
                fds[2].events = POLLIN;
                fds[3].fd = _uuv_one_pose_sub;
                fds[3].events = POLLIN;
                fds[4].fd = _uuv_two_pose_sub;
                fds[4].events = POLLIN;
                fds[5].fd = _uuv_three_pose_sub;
                fds[5].events = POLLIN;
                break;
            case 4:
                fds[0].fd = _sensor_sub;
                fds[0].events = POLLIN;
                fds[1].fd = _vehicle_attitude_sub;
                fds[1].events = POLLIN;
                fds[2].fd = _vehicle_local_position_sub;
                fds[2].events = POLLIN;
                fds[3].fd = _uuv_one_pose_sub;
                fds[3].events = POLLIN;
                fds[4].fd = _uuv_two_pose_sub;
                fds[4].events = POLLIN;
                fds[5].fd = _uuv_three_pose_sub;
                fds[5].events = POLLIN;
                fds[6].fd = _uuv_four_pose_sub;
                fds[6].events = POLLIN;
                break;
            case 5:
                fds[0].fd = _sensor_sub;
                fds[0].events = POLLIN;
                fds[1].fd = _vehicle_attitude_sub;
                fds[1].events = POLLIN;
                fds[2].fd = _vehicle_local_position_sub;
                fds[2].events = POLLIN;
                fds[3].fd = _uuv_one_pose_sub;
                fds[3].events = POLLIN;
                fds[4].fd = _uuv_two_pose_sub;
                fds[4].events = POLLIN;
                fds[5].fd = _uuv_three_pose_sub;
                fds[5].events = POLLIN;
                fds[6].fd = _uuv_four_pose_sub;
                fds[6].events = POLLIN;
                fds[7].fd = _uuv_five_pose_sub;
                fds[7].events = POLLIN;
                break;
        }




        int error_counter = 0;
        double phi_target = 0;
        double phi_act;
        double alpha=0;
        double beta=0;
        double r2x=0;
        double Ldelr=0;
        double theta_act=0;//, theta_bef=0;
        double theta_target;
        double Kpf=2;
        double Kpro = 5;
        double Kdf = 4;
        double Kdro = 4;
        double Kif = 1;
        double Kiro = 3;
        double nu;                  // steering controller
        double mu;                  // pitch contoller
        double eta;                 // roll controller
        double dt0=0, dt1=0;        // Steptimedifference
        double ro, p, y=0, t;//, roa, pa, ya, ta, regmax;          //roll pitch yaw trhrust parameters.
        double f1=0, f0=0;          // Errors phi (0: state before, 1: actual error)
        double ro1=0, ro0=0;        // Errors eta (0: state before, 1: actual error)
        double e1=0, e0=0;
        double de=0;                // Error differende yawspeed
        double df=0;                // Error difference phi
        double dro=0;               // Error difference eta
        double Ie=0;                // indegrated Error yawspeed
        double If=0;                // integrated Error phi
        double Iro=0;               // integrated Error eta



        matrix::Vector3<double> T0;
        matrix::Vector3<double> T;

        matrix::Vector3<double> x_B(0, 0, 0);     // orientation body x-axis (in world coordinates)
        matrix::Vector3<double> y_B(0, 0, 0);     // orientation body y-axis (in world coordinates)
        matrix::Vector3<double> z_B(0, 0, 0);     // orientation body z-axis (in world coordinates)

        matrix::Vector3<double> r(0, 0, 0);       // local position vector

        matrix::Vector3<double> T1(0, 0, 0);       // Position Vektor of other Boats
        matrix::Vector3<double> T2(0, 0, 0);       // Position Vektor of other Boats
        matrix::Vector3<double> T3(0, 0, 0);       // Position Vektor of other Boats
        matrix::Vector3<double> T4(0, 0, 0);       // Position Vektor of other Boats
        matrix::Vector3<double> T5(0, 0, 0);       // Position Vektor of other Boats

        matrix::Vector3<double> RT(0,0,0);        // nearest point on Tajectory in global coordinates
        matrix::Vector3<double> Rtarget(0,0,0);   // Target vector
        matrix::Vector3<double> rctr(0,0,0);      // direction to Rtarget from boat in global coordinates
        matrix::Vector3<double> delr(0,0,0);      // controll help

        while(_params.shouldRun==0){
            parameters_update();
            dt1=hrt_absolute_time()/(double)1000000;
        }

        while(!_task_should_exit && _params.shouldRun==1) {

            usleep(50000);

                // next step
                dt0 = dt1;
                dt1=hrt_absolute_time()/(double)1000000; // actual steptime
                f0=f1;
                ro0=ro1;
                e0=e1;

                /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
                int poll_ret = px4_poll(fds, 1, 1000);

                /* handle the poll result */
                if (poll_ret == 0) {
                        /* this means none of our providers is giving us data */
                        PX4_ERR("Got no data within a second");

                } else if (poll_ret < 0) {
                        /* this is seriously bad - should be an emergency */
                        if (error_counter < 10 || error_counter % 50 == 0) {
                                /* use a counter to prevent flooding (and slowing us down) */
                                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
                        }

                        error_counter++;

                } else {

                        if (fds[0].revents & POLLIN) {
                                /* obtained data for the second file descriptor */
                                struct vehicle_attitude_s raw_ctrl_state;
                                /* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &raw_ctrl_state);

                                // get current rotation matrix from control state quaternions, the quaternions are generated by the
                                // attitude_estimator_q application using the sensor data
                                matrix::Quatf q_att(raw_ctrl_state.q);     // control_state is frequently updated
                                matrix::Dcmf R = q_att; // create rotation matrix for the quaternion when post multiplying with a column vector

                                // orientation vectors
                                x_B(0)=R(1, 0);
                                x_B(1)=R(0, 0);
                                x_B(2)=-R(2, 0);     // orientation body x-axis (in world coordinates)
                                y_B(0)=R(1, 1);
                                y_B(1)=R(0, 1);
                                y_B(2)=-R(2, 1);     // orientation body y-axis (in world coordinates)
                                z_B(0)=R(1, 2);
                                z_B(1)=R(0, 2);
                                z_B(2)=-R(2, 2);     // orientation body z-axis (in world coordinates)

                                T(0)=T0(0)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));
                                T(1)=T0(1)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));
                                T(2)=T0(2)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));


                                /*PX4_INFO("x_B:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)x_B(0),
                                         (double)x_B(1),
                                         (double)x_B(2));

                                PX4_INFO("y_B:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)y_B(0),
                                         (double)y_B(1),
                                         (double)y_B(2));

                                PX4_INFO("z_B:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)z_B(0),
                                         (double)z_B(1),
                                         (double)z_B(2));
*/

                                /* obtained data for the third file descriptor */
                                struct vehicle_local_position_s raw_position;
                                /* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &raw_position);
                                r(0)=raw_position.y;
                                r(1)=raw_position.x;
                                r(2)=-raw_position.z;

                                struct uuv_one_pose_s uuv1pos;
                                orb_copy(ORB_ID(uuv_one_pose), _uuv_one_pose_sub, &uuv1pos);
                                T1(0)=uuv1pos.y;
                                T1(1)=uuv1pos.x;
                                T1(2)=-uuv1pos.z;
                                struct uuv_two_pose_s uuv2pos;
                                orb_copy(ORB_ID(uuv_two_pose), _uuv_two_pose_sub, &uuv2pos);
                                T2(0)=uuv2pos.y;//+1;
                                T2(1)=uuv2pos.x;
                                T2(2)=-uuv2pos.z;
                                struct uuv_three_pose_s uuv3pos;
                                orb_copy(ORB_ID(uuv_three_pose), _uuv_three_pose_sub, &uuv3pos);
                                T3(0)=uuv3pos.y;//+2;
                                T3(1)=uuv3pos.x;
                                T3(2)=-uuv3pos.z;
                                struct uuv_four_pose_s uuv4pos;
                                orb_copy(ORB_ID(uuv_four_pose), _uuv_four_pose_sub, &uuv4pos);
                                T4(0)=uuv4pos.y;//+3;
                                T4(1)=uuv4pos.x;
                                T4(2)=-uuv4pos.z;
                                struct uuv_five_pose_s uuv5pos;
                                orb_copy(ORB_ID(uuv_five_pose), _uuv_five_pose_sub, &uuv5pos);
                                T5(0)=uuv5pos.y;//+4;
                                T5(1)=uuv5pos.x;
                                T5(2)=-uuv5pos.z;




                        }
                }

                // Actual Boat-Headings
                phi_act=atan2(x_B(2),sqrt(pow(x_B(0),2)+pow(x_B(1),2)));            // angle between global XY-Plane and Boat-X-Axis
                theta_act=atan2(x_B(1),x_B(0));                                     // angle between global and Boat X-Axis

                if (_params.Order==1){
                    PX4_INFO("LEADER");

                    // Actualize Trajectory
                    T0(0)= _params.T_x;
                    T0(1)= _params.T_y;
                    T0(2)= _params.T_z;

                    T(0)=T0(0)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));
                    T(1)=T0(1)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));
                    T(2)=T0(2)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));

                    //Trajectory direction angles
                    alpha=atan2(T(1),T(0));                                             // Angle between global X-Axis and Trajectory Projection in X-Y-Plane
                    beta=atan2(T(2),sqrt(pow(T(0),2)+pow(T(1),2)));                     // Angle between global XY-Plane and Trajectory

                    // nearest Point on Trajectory in Trajectory coordinates
                    r2x=r(0)*cos(alpha)*cos(beta)+r(1)*sin(alpha)*cos(beta)+r(2)*sin(beta);

                    // nearest Point on Trajectory in global coordinates
                    RT(0)=_params.Toff_x + r2x*cos(alpha)*cos(beta);
                    RT(1)=_params.Toff_y + r2x*sin(alpha)*cos(beta);
                    RT(2)=_params.Toff_z + r2x*sin(beta);

                    // controller target Point
                    Rtarget = RT+_params.Kt*T;

                    // controller direction Vector
                    rctr=Rtarget-r;

                    // nearest distance Vector to trajectory
                    delr = RT-r;                                                        // Vector
                    Ldelr = sqrt(pow(delr(0),2)+pow(delr(1),2));//+pow(delr(2),2));         // Distance

                }else if(_params.Order>1){
                    PX4_INFO("Follower");
                    if(_params.IfChain==0){
                        switch(_params.Order){
                            case 2:
                                rctr = T1-T2;
                                break;
                            case 3:
                                rctr = T1-T3;
                                break;
                            case 4:
                                rctr = T1-T4;
                                break;
                            case 5:
                                rctr = T1-T5;
                                break;
                        }
                    }else{
                        switch(_params.Order){
                            case 2:
                                rctr = T1-T2;
                                break;
                            case 3:
                                rctr = T2-T3;
                                break;
                            case 4:
                                rctr = T3-T4;
                                break;
                            case 5:
                                rctr = T4-T5;
                                break;

                        }
                    }
                    // Distance to leader
                    Ldelr = sqrt(pow(rctr(0),2)+pow(rctr(1),2));//+pow(rctr(2),2));         // Distance
                }

                // Controller Target Angles of rctr
                theta_target = atan2(rctr(1),rctr(0));                              // angle between global X-Axis and rctr
                phi_target = atan2(rctr(2),sqrt(pow(rctr(0),2)+pow(rctr(1),2)));    // angle betreen global XY-Plane and rctr


                e1 = sin(theta_target-theta_act);                                   // Yaw
                f1 = sin(phi_target-phi_act);                                       // Pitch
                ro1 = sin(3.1415-atan2(y_B(2),sqrt(pow(y_B(0),2)+pow(y_B(1),2))));  // Roll

                // Differentiations
                de = (e1-e0)/(dt1-dt0);
                df = (f1-f0)/(dt1-dt0);
                dro = (ro1-ro0)/(dt1-dt0);

                // Integrations
                Ie += e1*(dt1-dt0);
                If += f1*(dt1-dt0);
                Iro+= ro1*(dt1-dt0);

                nu = _params.Kpy*e1+_params.Kiy*Ie+_params.Kdy*de;
                mu = Kpf*f1+Kif*If+Kdf*df;
                eta = Kpro*ro1+Kiro*Iro+Kdro*dro;


                // Controller arguments transformed in Boat-Coordinates
                ro= eta;//(eta-nu*x_B(2));
                p= mu;//mu-nu*y_B(2);
                y= -nu;//-nu*-z_B(2);
                if (_params.Order==1){
                    t= _params.Ksp;
                    //t = (_params.Ksp*(1+Ldelr));//ro/3+p/3+y/3;
                }else{
                    if (Ldelr<1){
                        t=_params.Ksp*(0.25+0.1*Ldelr);
                    }else{
                        t = _params.Ksp*(0.25+0.2*Ldelr);
                    }
                }

                // Give actuator input to the HippoC
                _actuators.control[0] = ro*0;         // roll
                _actuators.control[1] = p*0;           // pitch
                _actuators.control[2] = y;           // yaw
                _actuators.control[3] = t;		// thrust
                actuators_publish();
                parameters_update();
                //parameter_update_poll();
        }


        PX4_INFO("Exiting uuv_leafo_app!");



}

int
UUVLeaFo::task_main_trampoline(int argc, char *argv[])
{
        leafo_app::g_leafo_app->task_main();
        return 0;
}

static void usage()
{
        errx(1, "usage: leafo_app {start|stop}");
}


int uuv_leafo_app_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage();
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (leafo_app::g_leafo_app != nullptr) {
                        errx(1, "already running");
                }

                leafo_app::g_leafo_app = new UUVLeaFo;

                if (leafo_app::g_leafo_app == nullptr) {
                        errx(1, "alloc failed");
                }

                if (OK != leafo_app::g_leafo_app->start()) {
                        delete leafo_app::g_leafo_app;
                        leafo_app::g_leafo_app = nullptr;
                        err(1, "start failed");
                }

                return 0;
        }

        if (leafo_app::g_leafo_app == nullptr) {
                errx(1, "not running");
        }

        if (!strcmp(argv[1], "stop")) {
                delete leafo_app::g_leafo_app;
                leafo_app::g_leafo_app = nullptr;

        } else {
                usage();
        }

        return 0;
}


