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
 * @file uuv_test_app.cpp
 *
 * This file let the hippocampus drive in a circle and prints the orientation as well as the acceleration data.
 * The HippoCampus is an autonomous underwater vehicle (AUV) designed by the Technical University Hamburg-Harburg (TUHH).
 * https://www.tuhh.de/mum/forschung/forschungsgebiete-und-projekte/flow-field-estimation-with-a-swarm-of-auvs.html
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
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/actuator_outputs.h>              // this topic gives the actuators control input
extern "C" __EXPORT int uuv_test_app_main(int argc, char *argv[]);

int uuv_test_app_main(int argc, char *argv[])
{
    /* subscribe to sensor_combined topic */
    int actuator_outputs_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));
    /* limit the update rate to 5 Hz */
    orb_set_interval(actuator_outputs_sub_fd, 200);

    /* advertise to actuator_control topic */
    struct actuator_controls_s act;
    memset(&act, 0, sizeof(act));
    orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[1] = {};
    fds[0].fd = actuator_outputs_sub_fd;
    fds[0].events = POLLIN;


    int error_counter = 0;
    double a,b,c,d;

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
                            /* obtained data for the second file descriptor */
                            struct actuator_outputs_s actout;
                            /* copy sensors raw data into local buffer */
                            orb_copy(ORB_ID(actuator_outputs), actuator_outputs_sub_fd, &actout);

                            a=actout.output[0];
                            b=actout.output[1];
                            c=actout.output[2];
                            d=actout.output[3];


                            PX4_INFO("out:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                                     (double)a,
                                     (double)b,
                                     (double)c,
                                     (double)d);


                            }
            }



            // Give actuator input to the HippoC
            act.control[0] = 0;//roa;         // roll
            act.control[1] = 0;//pa;           // pitch
            act.control[2] = 0;//ya;           // yaw
            act.control[3] = -1;//ta;		// thrust
            orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);

    }


    PX4_INFO("Exiting uuv_example_app!");


    return 0;
}


