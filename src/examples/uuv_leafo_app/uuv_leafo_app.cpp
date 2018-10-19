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

// system libraries
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>

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
extern "C" __EXPORT int uuv_leafo_app_main(int argc, char *argv[]);

int uuv_leafo_app_main(int argc, char *argv[])
{


        PX4_INFO("auv_hippocampus_leader_app has been started!");

        /* subscribe to sensor_combined topic */
        int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
        /* limit the update rate to 5 Hz */
        orb_set_interval(sensor_sub_fd, 200);

        /* subscribe to control_state topic */
        int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
        /* limit the update rate to 5 Hz */
        orb_set_interval(vehicle_attitude_sub_fd, 200);

        /* subscribe to localization topic */
        int vehicle_local_position_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
        /* limit the update rate to 5 Hz */
        orb_set_interval(vehicle_local_position_sub_fd, 200);

        /* advertise to actuator_control topic */
        struct actuator_controls_s act;
        memset(&act, 0, sizeof(act));
        orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

        /* one could wait for multiple topics with this technique, just using one here */
        px4_pollfd_struct_t fds[3] = {};
        fds[0].fd = sensor_sub_fd;
        fds[0].events = POLLIN;
        fds[1].fd = vehicle_attitude_sub_fd;
        fds[1].events = POLLIN;
        fds[2].fd = vehicle_local_position_sub_fd;
        fds[2].events = POLLIN;



    //orb_publish_auto(ORB_ID(vehicle_vision_position), &_vision_position_pub, &vision_position, &inst, ORB_PRIO_DEFAULT);


        int error_counter = 0;
        double phi_target;
        double phi_act;
        double theta_target;
        double theta_act;
        double alpha;               // angle between X and XY-Projection ov Trajectory
        double beta;                // angle between Trajectory an XY-Plane
        double r2x;                 // nearest point aon trajectory in local coordinates
        double Kt = 7.5;            // Softness Gain (Trajectory controll Target Point)
        double Kpe =5;              // proportional Gain theta
        double Kpf =2;              // proportional Gain phi
        double Kpro =5;             // proportional Gain eta
        double Kie =0.2;              // integrator Gain theta
        double Kif =2;              // integrator Gain phi
        double Kiro =3;             // integrator Gain eta
        double Kde =10;              // differentiator Gain theta
        double Kdf =4;              // differentiator Gain phi
        double Kdro =4;             // differentiator Gain eta
        double Ksp= 0.5;         // speed Gain
        double nu;                  // yaw controller
        double mu;                  // pitch contoller
        double eta;                 // roll controller
        double dt0=0, dt1=0;        // Steptimedifference
        double Ldelr;               // Distance from Boat to target Point
        double ro, p, y, t;//, roa, pa, ya, ta, regmax;          //roll pitch yaw trhrust parameters.
        double e1=0, e0=0;          // Errors theta (0: state before, 1: actual error)
        double f1=0, f0=0;          // Errors phi (0: state before, 1: actual error)
        double ro1=0, ro0=0;        // Errors eta (0: state before, 1: actual error)
        double de=0;                // Error difference theta
        double df=0;                // Error difference phi
        double dro=0;               // Error difference eta
        double Ie=0;                // Integrated Error theta
        double If=0;                // integrated Error phi
        double Iro=0;               // integrated Error eta

        //Trajectory to plan:
        matrix::Vector3<double> T0(1,0,0);
        matrix::Vector3<double> T;
        PX4_INFO("T:\t%8.4f\t%8.4f\t%8.4f",
                 (double)T0(0),
                 (double)T0(1),
                 (double)T0(2));
        T(0)=T0(0)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));
        T(1)=T0(1)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));
        T(2)=T0(2)/sqrt(pow(T0(0),2)+pow(T0(1),2)+pow(T0(2),2));
        PX4_INFO("T:\t%8.4f\t%8.4f\t%8.4f",
                 (double)T(0),
                 (double)T(1),
                 (double)T(2));
        //Trajectory direction angles
        alpha=atan2(T(1),T(0));                                             // Angle between global X-Axis and Trajectory Projection in X-Y-Plane
        beta=atan2(T(2),sqrt(pow(T(0),2)+pow(T(1),2)));                     // Angle between global XY-Plane and Trajectory

        matrix::Vector3<double> x_B(0, 0, 0);     // orientation body x-axis (in world coordinates)
        matrix::Vector3<double> y_B(0, 0, 0);     // orientation body y-axis (in world coordinates)
        matrix::Vector3<double> z_B(0, 0, 0);     // orientation body z-axis (in world coordinates)

        matrix::Vector3<double> r(0, 0, 0);       // local position vector

        matrix::Vector3<double> RT(0,0,0);        // nearest point on Tajectory in global coordinates

        matrix::Vector3<double> Rtarget(0,0,0);   // Target vector

        matrix::Vector3<double> rctr(0,0,0);      // direction to Rtarget from boat in global coordinates

        matrix::Vector3<double> delr(0,0,0);      // controll help


   for (int i = 0; i < 300; i++) {
                // next step
                dt0 = dt1;
                e0=e1;
                f0=f1;
                ro0=ro1;

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
                                /* obtained data for the first file descriptor */
                                struct sensor_combined_s raw_sensor;
                                /* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw_sensor);
                                // printing the sensor data into the terminal
/*                              PX4_INFO("Acc:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)raw_sensor.accelerometer_m_s2[0],
                                         (double)raw_sensor.accelerometer_m_s2[1],
                                         (double)raw_sensor.accelerometer_m_s2[2]);
*/
                                /* obtained data for the second file descriptor */
                                struct vehicle_attitude_s raw_ctrl_state;
                                /* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &raw_ctrl_state);

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


                                PX4_INFO("x_B:\t%8.4f\t%8.4f\t%8.4f",
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


                                /* obtained data for the third file descriptor */
                                struct vehicle_local_position_s raw_position;
                                /* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub_fd, &raw_position);
                                r(0)=raw_position.y;
                                r(1)=raw_position.x;
                                r(2)=-raw_position.z;

                                dt1=(double)raw_position.timestamp/(double)1000000; // actual steptime
                                if(i==0){
                                    dt0=dt1;
                                }
                                // printing the sensor data into the terminal
                                PX4_INFO("POS:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)r(0),
                                         (double)r(1),
                                         (double)r(2));
                                // local position Vector r in global coordinates



                        }
                }

                // Actual Boat-Heading
                phi_act=atan2(x_B(2),sqrt(pow(x_B(0),2)+pow(x_B(1),2)));            // angle between global XY-Plane and Boat-X-Axis
                theta_act=atan2(x_B(1),x_B(0));                                     // angle between global and Boat X-Axis

                // nearest Point on Trajectory in Trajectory coordinates
                r2x=r(0)*cos(alpha)*cos(beta)+r(1)*sin(alpha)*cos(beta)+r(2)*sin(beta);

                // nearest Point on Trajectory in global coordinates
                RT(0)=r2x*cos(alpha)*cos(beta);
                RT(1)=r2x*sin(alpha)*cos(beta);
                RT(2)=r2x*sin(beta);

/*
                PX4_INFO("RT:\t%8.4f\t%8.4f\t%8.4f",
                         (double)RT(0),
                         (double)RT(1),
                         (double)RT(2));
*/
                // controller target Point
                Rtarget = RT+Kt*T;
/*
                PX4_INFO("Rtarget:\t%8.4f\t%8.4f\t%8.4f",
                         (double)Rtarget(0),
                         (double)Rtarget(1),
                         (double)Rtarget(2));
*/
                // controller direction Vector
                rctr=Rtarget-r;
/*
                PX4_INFO("rctr:\t%8.4f\t%8.4f\t%8.4f",
                         (double)rctr(0),
                         (double)rctr(1),
                         (double)rctr(2));
*/
                // nearest distance Vector to trajectory
                delr = RT-r;                                                        // Vector
                Ldelr = sqrt(pow(delr(0),2)+pow(delr(1),2)+pow(delr(2),2));         // Distance

                // Controller Target Angles of rctr
                theta_target = atan2(rctr(1),rctr(0));                              // angle between global X-Axis and rctr
                phi_target = atan2(rctr(2),sqrt(pow(rctr(0),2)+pow(rctr(1),2)));    // angle betreen global XY-Plane and rctr

/*
                PX4_INFO("phi_act:\t%8.4f",
                         (double)phi_act);
                PX4_INFO("phi_target:\t%8.4f",
                         (double)phi_target);
                PX4_INFO("theta_act:\t%8.4f",
                         (double)theta_act);
                PX4_INFO("theta_target:\t%8.4f",
                         (double)theta_target);
                PX4_INFO("theta-theta:\t%8.4f",
                         (double)theta_target-theta_act);
*/                // actual errors
                if ((theta_target-theta_act)>1.5){
                    e1=1;
                }
                else if ((theta_target-theta_act)<-1.5){
                    e1=-1;
                }
                else{
                    e1 = sin(theta_target-theta_act);                                   // Yaw
                }
                f1 = sin(phi_target-phi_act);                                       // Pitch
                ro1 = sin(3.1415-atan2(y_B(2),sqrt(pow(y_B(0),2)+pow(y_B(1),2))));  // Roll
                //PX4_INFO("e1:\t%8.4f",
                //         (double)e1);
                // Differentiations
                de = (e1-e0)/(dt1-dt0);
                df = (f1-f0)/(dt1-dt0);
                dro = (ro1-ro0)/(dt1-dt0);

                // Integrations
                Ie += e1*(dt1-dt0);
                If += f1*(dt1-dt0);
                Iro+= ro1*(dt1-dt0);

                if(i<=2){
                    nu =0;
                    mu=0;
                    eta=0;
                }
                else{
                nu = Kpe*e1+Kie*Ie+Kde*de;
                mu = Kpf*f1+Kif*If+Kdf*df;
                eta = Kpro*ro1+Kiro*Iro+Kdro*dro;
                }

                // Controller arguments transformed in Boat-Coordinates
                ro= eta;//(eta-nu*x_B(2));
                p= mu;//mu-nu*y_B(2);
                y= -nu;//-nu*-z_B(2);
                t= (Ksp*(1+Ldelr));//ro/3+p/3+y/3;


                // Equalized Controller Arguments
                // regmax = sqrt(ro*ro)+sqrt(p*p)+sqrt(y*y)+sqrt(t*t);
                // roa= 0.75*ro/regmax;
                // pa= 0.75*p/regmax;
                // ya= 0.75*y/regmax;
                // ta= 0.25;//t/regmax;

                PX4_INFO("Ldelr:\t%8.4f",
                         (double)Ldelr);
                PX4_INFO("ro:\t%8.4f",
                         (double)ro);
                PX4_INFO("p:\t%8.4f",
                         (double)p);
                PX4_INFO("y:\t%8.4f",
                         (double)y);
                PX4_INFO("t:\t%8.4f \n",
                         (double)t);


                // Give actuator input to the HippoC
                act.control[0] = ro*0;         // roll
                act.control[1] = p*0;           // pitch
                act.control[2] = y;           // yaw
                act.control[3] = t;		// thrust
                orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);

        }


        PX4_INFO("Exiting uuv_leader_app!");


        return 0;
}



