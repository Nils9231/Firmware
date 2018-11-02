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
 * @file uuv_mixer_balancer.cpp
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
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/actuator_outputs.h>               // this topic gives the actuators outputs




extern "C" __EXPORT int uuv_mixer_balancer_main(int argc, char *argv[]);

class UUVBalance
{
public:
    UUVBalance();
    ~UUVBalance();
     int start();
private:
     bool		_task_should_exit;		/**< if true, task should exit */
     int		_main_task;			/**< handle for task */
     orb_advert_t	_mavlink_log_pub;
     orb_advert_t	_actuators_pub;
     int                _params_sub;
     int                _actuator_outputs_sub;
     int                _actuator_controls_sub;
     struct actuator_outputs_s _actuators;


     struct{
         param_t thrust_factor;               // Number of Vehicles
         param_t delta_out_max;

     } _params_handles;

     struct{
         float thrust_factor;
         float delta_out_max;
     } _params;

     int		actuators_publish();

     int                parameters_update();

     void               parameter_update_poll();

     void               task_main();

     static int	task_main_trampoline(int argc, char *argv[]);


};


namespace uuvbalancer
{
UUVBalance	*g_uuvbalancer;
}

UUVBalance::UUVBalance():

    _task_should_exit(false),
    _main_task(-1),
    _mavlink_log_pub(nullptr),
    _actuators_pub(nullptr),
    _params_sub(-1),
    _actuators {}
{
    // define publication settings
    memset(&_actuators, 0, sizeof(_actuators));

        // allocate parameter handles
    _params_handles.thrust_factor         = param_find("UUV_TF");       // Number of Vehicles in Cicle
    _params_handles.delta_out_max         = param_find("UUV_OUT_MAX");



    // fetch initial parameter values
    parameters_update();
}


UUVBalance::~UUVBalance()
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

      uuvbalancer::g_uuvbalancer = nullptr;
}

int UUVBalance::parameters_update()
{
        float v;

        param_get(_params_handles.thrust_factor, &v);
        _params.thrust_factor = v;
        param_get(_params_handles.delta_out_max, &v);
        _params.delta_out_max = v;



        return OK;
}


int
UUVBalance::start()
{
    ASSERT(_main_task == -1);

    /* start the task */
    _main_task = px4_task_spawn_cmd("uuvbalancer",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT + 15,
                                    1500,
                                    (px4_main_t)&UUVBalance::task_main_trampoline,
                                    nullptr);

    if (_main_task < 0) {
            warn("task start failed");
            return -errno;
    }

    return OK;
}

int
UUVBalance::actuators_publish()
{
        _actuators.timestamp = hrt_absolute_time();

        // lazily publish _actuators only once available
        if (_actuators_pub != nullptr) {
                return orb_publish(ORB_ID(actuator_outputs), _actuators_pub, &_actuators);


        } else {
                _actuators_pub = orb_advertise(ORB_ID(actuator_outputs), &_actuators);

                if (_actuators_pub != nullptr) {
                        return OK;

                } else {
                        return -1;
                }
        }
}



void
UUVBalance::task_main()
{

        mavlink_log_info(&_mavlink_log_pub, "[uuvbalancer] has been started!");

       // PX4_INFO("auv_hippocampus_uuvbalancer has been started!");

        /* subscribe to actuator_controls topic */
        _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
        /* subscribe to actuator_controls topic */
        _actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));


        // wakeup source(s)
        px4_pollfd_struct_t fds[2];
        fds[0].fd = _actuator_controls_sub;
        fds[0].events = POLLIN;
        fds[1].fd = _actuator_outputs_sub;
        fds[1].events = POLLIN;
             
        int error_counter = 0;

        float min_out = 1.0;
        float max_out = -1.0;
        float out[4];
        float out_prev[4];

      

        

        while(!_task_should_exit) {

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
                        struct actuator_controls_s controls;
                        orb_copy(ORB_ID(actuator_controls), _actuator_controls_sub, &controls);
                        float roll = controls.control[0];
                        float pitch = controls.control[1];
                        float yaw = controls.control[2];
                        float thrust = controls.control[3];

                        out[0]=0.0f-pitch+yaw+thrust;
                        out[1]=0.0f-pitch-yaw+thrust;
                        out[2]=0.0f+pitch-yaw+thrust;
                        out[3]=0.0f+pitch+yaw+thrust;

                        for(unsigned i =0; i<4; i++){
                            if (out[i] < min_out) {
                                min_out = out[i];
                            }

                            if (out[i] > max_out) {
                                max_out = out[i];
                            }
                        }

                        float boost = 0.0f;		// value added to demanded thrust (can also be negative)
                        float yaw_pitch_scale = 1.0f;	// scale for demanded roll and pitch
                        float delta_out_max = max_out - min_out; // distance between the two extrema

                        if (delta_out_max <= 2.0f) {
                                if (min_out < -1.0f) {
                                        boost = -(min_out + 1.0f);

                                } else if (max_out > 1.0f) {
                                        boost = -(max_out - 1.0f);
                                }

                        } else {
                                yaw_pitch_scale = 1.0f / (delta_out_max);
                                boost = 1.0f - ((max_out - thrust) * yaw_pitch_scale + thrust);
                        }

                        float thrust_reduction = 0.0f;

                        out[0]=0.0f-roll+(0.0f-pitch+yaw)*yaw_pitch_scale+thrust+boost;
                        out[1]=0.0f+roll+(0.0f-pitch-yaw)*yaw_pitch_scale+thrust+boost;
                        out[2]=0.0f-roll+(0.0f+pitch-yaw)*yaw_pitch_scale+thrust+boost;
                        out[3]=0.0f+roll+(0.0f+pitch+yaw)*yaw_pitch_scale+thrust+boost;

                        for (unsigned i = 0; i < 4; i++) {
                                if (out[i] < -1.0f) {
                                        roll = -((yaw + pitch) * yaw_pitch_scale + thrust + boost);
                                } else if (out[i] > 1.0f) {
                                        // allow to reduce thrust to get some yaw response
                                        float prop_reduction = fminf(0.15f, out[i] - 1.0f);
                                        // keep the maximum requested reduction
                                        thrust_reduction = fmaxf(thrust_reduction, prop_reduction);
                                        roll = 1.0f -((yaw + pitch) * yaw_pitch_scale + thrust - thrust_reduction + boost);
                                }
                        }

                        thrust -= thrust_reduction;

                        out[0]=0.0f-roll+(0.0f-pitch+yaw)*yaw_pitch_scale+thrust+boost;
                        out[1]=0.0f+roll+(0.0f-pitch-yaw)*yaw_pitch_scale+thrust+boost;
                        out[2]=0.0f-roll+(0.0f+pitch-yaw)*yaw_pitch_scale+thrust+boost;
                        out[3]=0.0f+roll+(0.0f+pitch+yaw)*yaw_pitch_scale+thrust+boost;

                        for (unsigned i = 0; i < 4; i++) {


                                /*
                                        implement simple model for static relationship between applied motor pwm and motor thrust
                                        model: thrust = (1 - _params.thrust_factor) * PWM + _params.thrust_factor * PWM^2
                                        this model assumes normalized input / output in the range [0,1] so this is the right place
                                        to do it as at this stage the outputs are in that range.
                                 */
                                if (_params.thrust_factor > 0.0f) {
                                        out[i] = -(1.0f - _params.thrust_factor) / (2.0f * _params.thrust_factor) + sqrtf((1.0f - _params.thrust_factor) *
                                                        (1.0f - _params.thrust_factor) / (4.0f * _params.thrust_factor * _params.thrust_factor) + (out[i] /
                                                                        _params.thrust_factor));
                                }



                        }

                        /* slew rate limiting and saturation checking */
                        for (unsigned i = 0; i < 4; i++) {

                                // check for saturation against slew rate limits
                                if (_params.delta_out_max > 0.0f) {
                                        float delta_out = out[i] - out_prev[i];

                                        if (delta_out > _params.delta_out_max) {
                                                out[i] = out_prev[i] + _params.delta_out_max;


                                        } else if (delta_out < -_params.delta_out_max) {
                                                out[i] = out_prev[i] - _params.delta_out_max;

                                        }
                                }

                                out_prev[i] = out[i];


                        }

                        // this will force the caller of the mixer to always supply new slew rate values, otherwise no slew rate limiting will happen
                        _params.delta_out_max = 0.0f;



                    }
            }


            PX4_INFO("out:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                    (double)out[0],
                    (double)out[1],
                    (double)out[2],
                    (double)out[3]);
            _actuators.output[0] = out[0];
            _actuators.output[1] = out[1];
            _actuators.output[2] = out[2];
            _actuators.output[3] = out[3];
            actuators_publish();
            parameters_update();



        }


        PX4_INFO("Exiting uuvbalancer!");



}

int
UUVBalance::task_main_trampoline(int argc, char *argv[])
{
        uuvbalancer::g_uuvbalancer->task_main();
        return 0;
}

static void usage()
{
        errx(1, "usage: uuvbalancer {start|stop}");
}


int uuv_mixer_balancer_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage();
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (uuvbalancer::g_uuvbalancer != nullptr) {
                        errx(1, "already running");
                }

                uuvbalancer::g_uuvbalancer = new UUVBalance;

                if (uuvbalancer::g_uuvbalancer == nullptr) {
                        errx(1, "alloc failed");
                }

                if (OK != uuvbalancer::g_uuvbalancer->start()) {
                        delete uuvbalancer::g_uuvbalancer;
                        uuvbalancer::g_uuvbalancer = nullptr;
                        err(1, "start failed");
                }

                return 0;
        }

        if (uuvbalancer::g_uuvbalancer == nullptr) {
                errx(1, "not running");
        }

        if (!strcmp(argv[1], "stop")) {
                delete uuvbalancer::g_uuvbalancer;
                uuvbalancer::g_uuvbalancer = nullptr;

        } else {
                usage();
        }

        return 0;
}


