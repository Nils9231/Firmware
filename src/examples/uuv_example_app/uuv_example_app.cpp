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
 * @file uuv_example_app.cpp
 *
 * This file let the hippocampus drive in a circle and prints the orientation as well as the acceleration data.
 * The HippoCampus is an autonomous underwater vehicle (AUV) designed by the Technical University Hamburg-Harburg (TUHH).
 * https://www.tuhh.de/mum/forschung/forschungsgebiete-und-projekte/flow-field-estimation-with-a-swarm-of-auvs.html
 *
 * @author Nils Rottann <Nils.Rottmann@tuhh.de>, Nils Timmermann <Nils.Timmermann@tuhh.de>
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
#include <uORB/topics/vehicle_local_position.h>         // this topic holds all positzion ans speed information
#include <uORB/topics/att_pos_mocap.h>
extern "C" __EXPORT int uuv_example_app_main(int argc, char *argv[]);

int uuv_example_app_main(int argc, char *argv[])
{
	PX4_INFO("auv_hippocampus_example_app has been started!");

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

        /* subscribe to localization topic */
        int att_pos_mocap_sub_fd = orb_subscribe(ORB_ID(att_pos_mocap));
        /* limit the update rate to 5 Hz */
        orb_set_interval(att_pos_mocap_sub_fd, 200);

	/* advertise to actuator_control topic */
	struct actuator_controls_s act;
	memset(&act, 0, sizeof(act));
	orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

	/* one could wait for multiple topics with this technique, just using one here */
        px4_pollfd_struct_t fds[4] = {};
	fds[0].fd = sensor_sub_fd;
	fds[0].events = POLLIN;
	fds[1].fd = vehicle_attitude_sub_fd;
	fds[1].events = POLLIN;
        fds[2].fd = vehicle_local_position_sub_fd;
        fds[2].events = POLLIN;
        fds[3].fd = att_pos_mocap_sub_fd;
        fds[3].events = POLLIN;


	int error_counter = 0;
        double phi_target;
        double phi_act;
        double theta_target;
        double theta_act;
        double alpha;               // angle between X and XY-Projection ov Trajectory
        double beta;                // angle between Trajectory an XY-Plane
        double r2x;                 // nearest point aon trajectory in local coordinates
        double Kp = -10;             // Gain vor collective oder destrictive motion. For Trajectory following it has to be Kp<0
        double Kt = 10;              // Softness Gain (Trajectory controll Target Point)
        //double Kpp=1;               // proportional Gain for second order steerer
        double Ksp=1;               // speed Gain
        double nu;
        double mu;
        double eta;
        //double omega;
        //double psi;
        double Ldelr;               // Distance from Boat to target Point

        //Trajectory to plan:
        matrix::Vector3d T(1,1,1);
        T(0)=T(0)/sqrt(pow(T(0),2)+pow(T(1),2)+pow(T(2),2));
        T(1)=T(1)/sqrt(pow(T(0),2)+pow(T(1),2)+pow(T(2),2));
        T(2)=T(2)/sqrt(pow(T(0),2)+pow(T(1),2)+pow(T(2),2));

        matrix::Vector3d x_B(0, 0, 0);     // orientation body x-axis (in world coordinates)
        matrix::Vector3d y_B(0, 0, 0);     // orientation body y-axis (in world coordinates)
        matrix::Vector3d z_B(0, 0, 0);     // orientation body z-axis (in world coordinates)

        matrix::Vector3d r(0, 0, 0);       // local position vector

        matrix::Vector3d RT(0,0,0);        // nearest point on Tajectory in global coordinates

        matrix::Vector3d Rtarget(0,0,0);   // Target vector

        matrix::Vector3d rctr(0,0,0);      // direction to Rtarget from boat in global coordinates

        matrix::Vector3d delr(0,0,0);      // controll help


    for (int i = 0; i < 25; i++) {
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
				PX4_INFO("Acc:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw_sensor.accelerometer_m_s2[0],
					 (double)raw_sensor.accelerometer_m_s2[1],
					 (double)raw_sensor.accelerometer_m_s2[2]);

                                /* obtained data for the second file descriptor */
				struct vehicle_attitude_s raw_ctrl_state;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &raw_ctrl_state);

				// get current rotation matrix from control state quaternions, the quaternions are generated by the
				// attitude_estimator_q application using the sensor data
                                matrix::Quatf q_att(raw_ctrl_state.q);     // control_state is frequently updated
                                matrix::Dcmf R = q_att; // create rotation matrix for the quaternion when post multiplying with a column vector

				// orientation vectors
                                x_B(0)=R(0, 0);
                                x_B(1)=R(1, 0);
                                x_B(2)=R(2, 0);     // orientation body x-axis (in world coordinates)
                                y_B(0)=R(0, 1);
                                y_B(1)=R(1, 1);
                                y_B(2)=R(2, 1);     // orientation body y-axis (in world coordinates)
                                z_B(0)=R(0, 2);
                                z_B(1)=R(1, 2);
                                z_B(2)=R(2, 2);     // orientation body z-axis (in world coordinates)

				PX4_INFO("x_B:\t%8.4f\t%8.4f\t%8.4f",
					 (double)x_B(0),
					 (double)x_B(1),
					 (double)x_B(2));

				PX4_INFO("y_B:\t%8.4f\t%8.4f\t%8.4f",
					 (double)y_B(0),
					 (double)y_B(1),
					 (double)y_B(2));

				PX4_INFO("z_B:\t%8.4f\t%8.4f\t%8.4f \n",
					 (double)z_B(0),
					 (double)z_B(1),
					 (double)z_B(2));

                                /* obtained data for the third file descriptor */
                                //struct vehicle_local_position_s raw_position;
                                /* copy sensors raw data into local buffer */
                                //orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub_fd, &raw_position);
                                /* obtained data for the third file descriptor */
                                struct att_pos_mocap_s raw_position;
                                /* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(att_pos_mocap), att_pos_mocap_sub_fd, &raw_position);
                                // printing the sensor data into the terminal
                                PX4_INFO("POS:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)raw_position.x,
                                         (double)raw_position.y,
                                         (double)raw_position.z);
                                // local position Vector r
                                r(0)=raw_position.x;
                                r(1)=raw_position.y;
                                r(2)=raw_position.z;
			}
		}

                //Trajectory Controller
                phi_act=atan2(x_B(2),sqrt(pow(x_B(0),2)+pow(x_B(1),2)));           //Actual heading
                theta_act=atan2(x_B(1),x_B(0));
                    //Trajectory direction angles
                alpha=atan2(T(1),T(0));                 //Angle between X-Axis and Trajectory Projection in X-Y-Plane
                beta=atan2(T(2),sqrt(pow(T(0),2)+pow(T(1),2)));   //Angle between X-Y-Plane and Trajectory

                r2x=r(0)*cos(alpha)*cos(beta)+r(1)*sin(alpha)*cos(beta)+r(2)*sin(beta); //nearest Point on Trajectory in Trajectory coordinates

                RT(0)=r2x*cos(alpha)*cos(beta);
                RT(1)=r2x*sin(alpha)*cos(beta);
                RT(2)=r2x*sin(beta);                    //nearest Point on Trajectory in global coordinates

                Rtarget = RT+Kt*T;

                rctr=Rtarget-r;

                delr = RT-r;
                Ldelr = sqrt(pow(delr(0),2)+pow(delr(1),2)+pow(delr(2),2));

                theta_target = atan2(rctr(1),rctr(0));
                phi_target = atan2(rctr(2),sqrt(pow(rctr(0),2)+pow(rctr(1),2)));

                nu = -Kp * sin(theta_target-theta_act);
                mu = -Kp * sin(phi_target-phi_act);
                eta= -sin(-atan2(y_B(2),sqrt(pow(y_B(0),2)+pow(y_B(1),2))));
                //R = transpose(R);


		// Give actuator input to the HippoC, this will result in a circle
                act.control[0] = -nu*x_B(2)+eta;         // roll
                act.control[1] = -nu*y_B(2)+mu;           // pitch
                act.control[2] = nu*z_B(2);           // yaw
                act.control[3] = (Ksp*(1+Ldelr));		// thrust
		orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);

	}


	PX4_INFO("Exiting uuv_example_app!");


	return 0;
}


