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
 * @file uuv_circ_app.cpp
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
#include <uORB/topics/position_setpoint.h>            // this Topic sets the Leader Position for the follower
#include <uORB/topics/parameter_update.h>


extern "C" __EXPORT int uuv_circ_app_main(int argc, char *argv[]);

class UUVCirc
{
public:
    UUVCirc();
    ~UUVCirc();
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
     int                _position_setpoint_sub;
     struct actuator_controls_s _actuators;


     struct{
         param_t NUM;               // Number of Vehicles
         param_t Order;             // Ordering Number of Vehicle
         param_t Kcirc;             // proportional Gain for first order Steerer
         param_t Kdes;              // proportional Gain for desired circ middlepoint in first order steerer;
         param_t Yconst;            // constant offset on Yaw_rate actuator input
         param_t Ksp;               // Speed constand
         param_t Kpy;               // proportional Gain Yaw_rate
         param_t Kpf;               // proportional Gain phi
         param_t Kpro;              // proportional Gain eta
         param_t Kiy;               // integrator Gain Yaw_rate
         param_t Kif;               // integrator Gain phi
         param_t Kiro;              // integrator Gain eta
         param_t Kdy;               // differentiator Gain Yaw_rate
         param_t Kdf;               // differentiator Gain phi
         param_t Kdro;              // differentiator Gain eta
         param_t cdes_x;            // desired Circle Middelpoint
         param_t cdes_y;
         param_t cdes_z;
         param_t ome0;              // desired angular speed
     } _params_handles;

     struct{
         int NUM;
         int Order;
         double Kcirc;              // proportional Gain for first order Steerer
         double Kdes;               // proportional Gain for desired circ middlepoint in first order steerer;
         double Yconst;             // constant offset on Yaw_rate actuator input
         double Ksp;                // speed constant
         double Kpy;                // proportional Gain Yaw_rate
         double Kpf;                // proportional Gain phi
         double Kpro;               // proportional Gain eta
         double Kiy;                // integrator Gain Yaw_rate
         double Kif;                // integrator Gain phi
         double Kiro;               // integrator Gain eta
         double Kdy;                // differentiator Gain Yaw_rate
         double Kdf;                // differentiator Gain phi
         double Kdro;               // differentiator Gain eta
         double cdes_x;             // desiresd circle Middlepoint
         double cdes_y;
         double cdes_z;
         double ome0;               // desired angular speed
     } _params;

     int		actuators_publish();

     int                parameters_update();

     void               parameter_update_poll();

     void               task_main();

     int                actuator_publish();

     static int	task_main_trampoline(int argc, char *argv[]);


};


namespace circ_app
{
UUVCirc	*g_circ_app;
}

UUVCirc::UUVCirc():

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
    _params_handles.NUM         = param_find("UUV_CIRC_NUM");       // Number of Vehicles in Cicle
    _params_handles.Order       = param_find("UUV_CIRC_Order");     // Ordering Number of Vehicle
    _params_handles.Kcirc       = param_find("UUV_CIRC_Kcirc");     // proportional Gain for first order Steerer
    _params_handles.Kdes        = param_find("UUV_CIRC_Kdes");      // proportional Gain for desired circ middlepoint in first order steerer;
    _params_handles.Yconst      = param_find("UUV_CIRC_Yconst");    // constant offset on Yaw_rate actuator input
    _params_handles.Ksp         = param_find("UUV_CIRC_Ksp");       // speed constant
    _params_handles.Kpy         = param_find("UUV_CIRC_Kpy");       // proportional Gain Yaw_rate
    _params_handles.Kpf         = param_find("UUV_CIRC_Kpf");       // proportional Gain phi
    _params_handles.Kpro        = param_find("UUV_CIRC_Kpro");      // proportional Gain eta
    _params_handles.Kiy         = param_find("UUV_CIRC_Kiy");       // integrator Gain Yaw_rate
    _params_handles.Kif         = param_find("UUV_CIRC_Kif");       // integrator Gain phi
    _params_handles.Kiro        = param_find("UUV_CIRC_Kiro");      // integrator Gain eta
    _params_handles.Kdy         = param_find("UUV_CIRC_Kdy");       // differentiator Gain Yaw_rate
    _params_handles.Kdf         = param_find("UUV_CIRC_Kdf");       // differentiator Gain phi
    _params_handles.Kdro        = param_find("UUV_CIRC_Kdro");      // differentiator Gain eta
    _params_handles.cdes_x      = param_find("UUV_CIRC_cdes_x");    //desired Circ-Middlepoint
    _params_handles.cdes_y      = param_find("UUV_CIRC_cdes_y");
    _params_handles.cdes_z      = param_find("UUV_CIRC_cdes_z");
    _params_handles.ome0        = param_find("UUV_CIRC_ome0");      // desired Angular speed


    // fetch initial parameter values
    parameters_update();
}


UUVCirc::~UUVCirc()
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

      circ_app::g_circ_app = nullptr;
}

int UUVCirc::parameters_update()
{
        float v;

        param_get(_params_handles.NUM, &v);
        _params.NUM = v;
        param_get(_params_handles.Order, &v);
        _params.Order = v;
        param_get(_params_handles.Kcirc, &v);
        _params.Kcirc = v;
        param_get(_params_handles.Kdes, &v);
        _params.Kdes = v;
        param_get(_params_handles.Yconst, &v);
        _params.Yconst = v;
        param_get(_params_handles.Ksp, &v);
        _params.Ksp = v;
        param_get(_params_handles.Kpy, &v);
        _params.Kpy = v;
        param_get(_params_handles.Kpf, &v);
        _params.Kpf = v;
        param_get(_params_handles.Kpro, &v);
        _params.Kpro = v;
        param_get(_params_handles.Kiy, &v);
        _params.Kiy = v;
        param_get(_params_handles.Kif, &v);
        _params.Kif = v;
        param_get(_params_handles.Kiro, &v);
        _params.Kiro = v;
        param_get(_params_handles.Kdy, &v);
        _params.Kdy = v;
        param_get(_params_handles.Kdf, &v);
        _params.Kdf = v;
        param_get(_params_handles.Kdro, &v);
        _params.Kdro = v;
        param_get(_params_handles.cdes_x, &v);
        _params.cdes_x = v;
        param_get(_params_handles.cdes_y, &v);
        _params.cdes_y = v;
        param_get(_params_handles.cdes_z, &v);
        _params.cdes_z = v;
        param_get(_params_handles.ome0, &v);
        _params.ome0 = v;



        return OK;
}

void UUVCirc::parameter_update_poll()
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
UUVCirc::start()
{
    ASSERT(_main_task == -1);

    /* start the task */
    _main_task = px4_task_spawn_cmd("circ_app",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT + 15,
                                    1500,
                                    (px4_main_t)&UUVCirc::task_main_trampoline,
                                    nullptr);

    if (_main_task < 0) {
            warn("task start failed");
            return -errno;
    }

    return OK;
}

int
UUVCirc::actuators_publish()
{
        _actuators.timestamp = hrt_absolute_time();

        // lazily publish _actuators only once available
        if (_actuator_pub != nullptr) {
                PX4_INFO("ACT:\t%8.4f\t%8.4f",
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
UUVCirc::task_main()
{

        mavlink_log_info(&_mavlink_log_pub, "[circ_app] has been started!");

       // PX4_INFO("auv_hippocampus_circ_app has been started!");

        /* subscribe to sensor_combined topic */
        _sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
        /* subscribe to control_state topic */
        _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        /* subscribe to localization topic */
        _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
        /* subscribe to position_setpoint topic */
        _position_setpoint_sub = orb_subscribe(ORB_ID(position_setpoint));


        // wakeup source(s)
        px4_pollfd_struct_t fds[4];

        // Setup of loop
        fds[0].fd = _sensor_sub;
        fds[0].events = POLLIN;
        fds[1].fd = _vehicle_attitude_sub;
        fds[1].events = POLLIN;
        fds[2].fd = _vehicle_local_position_sub;
        fds[2].events = POLLIN;
        fds[3].fd = _position_setpoint_sub;
        fds[3].events = POLLIN;

        int error_counter = 0;
        double phi_target = 0;
        double phi_act;
        double theta_act=0;//, theta_bef=0;
        double theta_bct=0;
        double psi=0;                 // yawspeed controller
        double nu;                  // steering controller
        double mu;                  // pitch contoller
        double eta;                 // roll controller
        double dt0=0, dt1=0;        // Steptimedifference
        double ro, p, y=0, t;//, roa, pa, ya, ta, regmax;          //roll pitch yaw trhrust parameters.
        //double y0=0,y1=0,y2=0,y3=0,y4=0;      //levelparameters
        double dyspd0=0, dyspd1=0;  // yawspeed error (0: state before, 1: actual error)
        double f1=0, f0=0;          // Errors phi (0: state before, 1: actual error)
        double ro1=0, ro0=0;        // Errors eta (0: state before, 1: actual error)
        double de=0;                // Error differende yawspeed
        double df=0;                // Error difference phi
        double dro=0;               // Error difference eta
        double Ie=0;                // indegrated Error yawspeed
        double If=0;                // integrated Error phi
        double Iro=0;               // integrated Error eta
        //double u;                   // second order steerer
        double yawspd=0;            // yawspeed



        matrix::Vector3<double> x_B(0, 0, 0);     // orientation body x-axis (in world coordinates)
        matrix::Vector3<double> y_B(0, 0, 0);     // orientation body y-axis (in world coordinates)
        matrix::Vector3<double> z_B(0, 0, 0);     // orientation body z-axis (in world coordinates)

        matrix::Vector3<double> r(0, 0, 0);       // local position vector
        matrix::Vector3<double> v1;               // local speed vectors
        //matrix::Vector3<double> v2;               // local speed vectors
        matrix::Vector3<double> T(0, 0, 0);       // Position Vektor of other Boats



        matrix::Vector3<double> c1;                // own circ-middlepoint vector
        matrix::Vector3<double> c2;                // friend circ-middlepoint vector
        matrix::Vector3<double> c3;                // desired circ-middlepoint vector


        while(!_task_should_exit) {
                // next step
                dt0 = dt1;
                dt1=hrt_absolute_time()/(double)1000000; // actual steptime
                f0=f1;
                ro0=ro1;
                dyspd0=dyspd1;
                //y4=y3;
                //y3=y2;
                //y2=y1;
                //y1=y0;
                //y0=y;

                //theta_bef=theta_act;

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
                                yawspd = raw_ctrl_state.yawspeed;

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



                                // printing the sensor data into the terminal
                                /*PX4_INFO("POS:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)r(0),
                                         (double)r(1),
                                         (double)r(2)); */
                                // local position Vector r in global coordinates
                                struct position_setpoint_s possp;
                                orb_copy(ORB_ID(position_setpoint), _position_setpoint_sub, &possp);
                                T(0)=possp.y+1;
                                T(1)=possp.x;
                                T(2)=-possp.z;
                                theta_bct=atan2(cos(possp.yaw),sin(possp.yaw));
                                //theta_bct = possp.yaw;
                                /*PX4_INFO("LPos:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                                         (double)T(0),
                                         (double)T(1),
                                         (double)T(2),
                                         (double)theta_bct);    */
                                // local position Vector T in global coordinates



                        }
                }

                // Actual Boat-Headings
                phi_act=atan2(x_B(2),sqrt(pow(x_B(0),2)+pow(x_B(1),2)));            // angle between global XY-Plane and Boat-X-Axis
                theta_act=atan2(x_B(1),x_B(0));                                     // angle between global and Boat X-Axis

                //Actual Velocity vectors
                v1(0)=cos(theta_act);
                v1(1)=sin(theta_act);
                v1(2)=0;
                //v2(0)=cos(theta_bct);
                //v2(1)=sin(theta_bct);
                //v2(2)=0;

                //circ middlepoints
                c1(0)=r(0)+sin(theta_act)/_params.ome0;
                c1(1)=r(1)-cos(theta_act)/_params.ome0;
                c1(2)=0;
                c2(0)=T(0)+sin(theta_bct)/_params.ome0;
                c2(1)=T(1)-cos(theta_bct)/_params.ome0;
                c2(2)=0;
                c3(0)=_params.cdes_x;
                c3(1)=_params.cdes_y;
                c3(2)=_params.cdes_z;



/*
                PX4_INFO("c1:\t%8.4f\t%8.4f\t%8.4f",
                         (double)c1(0),
                         (double)c1(1),
                         (double)c1(2));
                PX4_INFO("c2:\t%8.4f\t%8.4f\t%8.4f",
                         (double)c2(0),
                         (double)c2(1),
                         (double)c2(2));
                PX4_INFO("c3:\t%8.4f\t%8.4f\t%8.4f",
                         (double)c3(0),
                         (double)c3(1),
                         (double)c3(2));
*/

                // steering controller
                nu = _params.ome0*(1+(1/(_params.NUM+_params.Kdes))*_params.Kcirc*((c1(0)-c2(0)-_params.Kdes*c3(0))*v1(0)+(c1(1)-c2(1)-_params.Kdes*c3(1))*v1(1)+(c1(2)-c2(2)-_params.Kdes*c3(2))*v1(2)));
                //u =Kp*(nu-yawspeed);
/*
                PX4_INFO("yawspd:\t%8.4f",
                         (double)yawspd);
*/
                //error definitions
                dyspd1 = nu-yawspd;
/*
                PX4_INFO("dyspd:\t%8.4f",
                         (double)dyspd1);
*/
                f1 = sin(phi_target-phi_act);                                       // Pitch
                ro1 = sin(3.1415-atan2(y_B(2),sqrt(pow(y_B(0),2)+pow(y_B(1),2))));  // Roll


                // Differentiations
                de = (dyspd1-dyspd0)/(dt1-dt0);
                df = (f1-f0)/(dt1-dt0);
                dro = (ro1-ro0)/(dt1-dt0);

                // Integrations
                Ie += dyspd1*(dt1-dt0);
                If += f1*(dt1-dt0);
                Iro+= ro1*(dt1-dt0);

                psi = _params.Kpy*dyspd1+_params.Kiy*Ie+_params.Kdy*de;
                mu = _params.Kpf*f1+_params.Kif*If+_params.Kdf*df;
                eta = _params.Kpro*ro1+_params.Kiro*Iro+_params.Kdro*dro;

                // Controller arguments transformed in Boat-Coordinates
                ro= eta;//(eta-nu*x_B(2));
                p= mu;//mu-nu*y_B(2);
                //y = Kold*(y0+y1+y2+y3+y4)+psi;//-nu*-z_B(2);
                y = _params.Yconst +psi;//-nu*-z_B(2);
                t= _params.Ksp;//ro/3+p/3+y/3;



                // Equalized Controller Arguments
                // regmax = sqrt(ro*ro)+sqrt(p*p)+sqrt(y*y)+sqrt(t*t);
                // roa= 0.75*ro/regmax;
                // pa= 0.75*p/regmax;
                // ya= 0.75*y/regmax;
                // ta= 0.25;//t/regmax;


                /*PX4_INFO("ro:\t%8.4f",
                         (double)ro);
                PX4_INFO("p:\t%8.4f",
                         (double)p);
                PX4_INFO("y:\t%8.4f",
                         (double)y);
                PX4_INFO("t:\t%8.4f \n",
                         (double)t);
*/

                // Give actuator input to the HippoC
                _actuators.control[0] = ro*0;         // roll
                _actuators.control[1] = p*0;           // pitch
                _actuators.control[2] = y;           // yaw
                _actuators.control[3] = t;		// thrust
                actuators_publish();

                parameter_update_poll();
        }


        PX4_INFO("Exiting uuv_circ_app!");



}

int
UUVCirc::task_main_trampoline(int argc, char *argv[])
{
        circ_app::g_circ_app->task_main();
        return 0;
}

static void usage()
{
        errx(1, "usage: circ_app {start|stop}");
}


int uuv_circ_app_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage();
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (circ_app::g_circ_app != nullptr) {
                        errx(1, "already running");
                }

                circ_app::g_circ_app = new UUVCirc;

                if (circ_app::g_circ_app == nullptr) {
                        errx(1, "alloc failed");
                }

                if (OK != circ_app::g_circ_app->start()) {
                        delete circ_app::g_circ_app;
                        circ_app::g_circ_app = nullptr;
                        err(1, "start failed");
                }

                return 0;
        }

        if (circ_app::g_circ_app == nullptr) {
                errx(1, "not running");
        }

        if (!strcmp(argv[1], "stop")) {
                delete circ_app::g_circ_app;
                circ_app::g_circ_app = nullptr;

        } else {
                usage();
        }

        return 0;
}


