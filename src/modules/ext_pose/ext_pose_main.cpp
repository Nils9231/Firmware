
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
 * @file ext_pose_main.cpp
 *
 * module pulls  data provided by external localization system e.g. EM-localization
 *
 * @author Viktor Rausch
 * @author Daniel Duecker
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
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/pressure.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
//#include <systemlib/systemlib.h>
//#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>
#include <uORB/topics/ext_2d_position.h>
#include <uORB/topics/adc_report.h> // includes ADC readings
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/debug_vect.h>



using namespace matrix;


extern "C" __EXPORT int ext_pose_main(int argc, char *argv[]);

class Read_EXT_Data {
public:
    /**
     * Constructor
     */
    Read_EXT_Data();

    /**
     * Destructor, also kills the main task
     */
    ~Read_EXT_Data();

    double n=1; //counter
    double k=1; //counter
    double l=1; //counter
    /**
     * Start the underwater attitude control task.
     *
     * @return		OK on success.
     */
    int		start();

private:
     bool	_task_should_exit;		/**< if true, task_main() should exit */
    int		_control_task;			/**< task handle */

    int   sd_save;
    float   water_depth;
    float      _roh_g;
    float      _p_zero;
    int         subscribed_ext_2d_position_int;
    int        _pressure_raw;
    int        _vehicle_local_position;
    int        _vehicle_attitude;
    int        counter;
    int         att_pos_mocap_int;
    float _dist_antenna;
    float _dist_pressure_sensor;

    orb_advert_t	att_pos_mocap_pub;		/**< attitude_pos_mocap publication */
    orb_advert_t	debug_vect_pub;		/**Visualization with QGroundControl */


    struct att_pos_mocap_s _mocap_vec;
    struct debug_vect_s _debug_vect;
    struct ext_2d_position_s subscribed_ext_2d_position;
    struct vehicle_attitude_s		    _v_att;		            // attitude data
    struct {
                param_t EXT_ALPHA_OFFSET;
                param_t EXT_TRANS_ENABLE;
	}		_params_handles;		// handling to find parameters

    struct {
                double alpha_offset;
                int trans_enable;
	}		_params;


       /**
     * Check for external ekf updates from Pi0 and handle it.
     */
    void		ext_update_poll();

    	// Update our local parameter cache.
	int			parameters_update();
    /**
     * Main attitude control task.
     */
    void		task_main();

      /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);
};

namespace ext_pose
{

 Read_EXT_Data	*g_control;
}


Read_EXT_Data::Read_EXT_Data() :


    _task_should_exit(false),
    _control_task(-1),
    subscribed_ext_2d_position_int(-1),
    debug_vect_pub(nullptr)

{
    memset(&_v_att, 0, sizeof(_v_att));
    _roh_g = 98.1;
    counter = 1;

 subscribed_ext_2d_position.ext_pos_x = 0;
 subscribed_ext_2d_position.ext_pos_y = 0;
 _mocap_vec.x = 0;
 _mocap_vec.y = 0;
 _mocap_vec.z = 0;
 _dist_antenna = 0.21;
 _dist_pressure_sensor = 0.21;
 sd_save = 0;

 _params_handles.EXT_ALPHA_OFFSET   = 	param_find("EXT_ALPHA_OFFSET");
 _params_handles.EXT_TRANS_ENABLE	= 	param_find("EXT_TRANS_ENABLE");

    	// fetch initial parameter values
	parameters_update();
}

Read_EXT_Data::~Read_EXT_Data()
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


    ext_pose::g_control = nullptr;
}

int Read_EXT_Data::parameters_update()
{
	float v;

        param_get(_params_handles.EXT_ALPHA_OFFSET, &v);
        _params.alpha_offset = v;
        param_get(_params_handles.EXT_TRANS_ENABLE, &v);
        _params.trans_enable = v;

	return OK;
}

void Read_EXT_Data::ext_update_poll()
{
    bool updated_press;
    bool updated_ext;

    orb_check( _pressure_raw, &updated_press);

    if (updated_press) {
                struct pressure_s press;

                /* get pressure value from sensor*/
                orb_copy(ORB_ID(pressure), _pressure_raw, &press);

                /* set surface air pressure  */
                if (counter == 1){
                    _p_zero = press.pressure_mbar;
                    counter = 0;
                }

                /* calculate actual water depth */
                water_depth = ( press.pressure_mbar - _p_zero ) / ( _roh_g ); //unit meter
    }

    //PX4_INFO("debug_waterdepth \t%8.4f", (double)water_depth);

    orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude, &_v_att);

    Matrix<float, 3, 3> R;             // actual rotation matrix
    // get current rotation matrix from control state quaternions, the quaternions are generated by the
    // attitude_estimator_q application using the sensor data
    matrix::Quatf q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
    // create rotation matrix for the quaternion when post multiplying with a column vector x'=R*x
    R = q_att.to_dcm();


    // orientation vectors
    Vector3f x_B(R(0, 0), R(1, 0), R(2,0));

    /* Check if parameters have changed */
    orb_check(subscribed_ext_2d_position_int, &updated_ext);

    if (updated_ext) {
        /* read from param to clear updated flag (uORB API requirement) */

        orb_copy(ORB_ID(ext_2d_position), subscribed_ext_2d_position_int, &subscribed_ext_2d_position);

        /*
        //Debug options
        PX4_INFO("Debug_Test:\t%.4f\t%.4f\n",
                       (double)subscribed_ekf_vector.EKF_pos_x,
                       (double)subscribed_ekf_vector.EKF_pos_y);
        */
        /*
        //save on sd card
        FILE *sd;
         if (sd_save ==0 ){
            sd = fopen("/fs/microsd/Position_Data.txt","w");
            fprintf(sd,"Mocap_Pos_Trans and EKF_Pos_Ant and rotation_vec:\n");
            fclose(sd);
            sd_save =1;
            }

            sd = fopen("/fs/microsd/Position_Data.txt","a");
            fprintf(sd,"\t%.4f\t%.4f\n",
            (double)subscribed_ekf_vector.EKF_pos_x,
            (double)subscribed_ekf_vector.EKF_pos_y);
            fclose(sd);
        */


        n=n+1;

        /*With transformation from antenna to bodycenter coordinates*/
        if (_params.trans_enable == 1){
            _mocap_vec.timestamp = hrt_absolute_time();
            _mocap_vec.x = subscribed_ext_2d_position.ext_pos_x/1000 + x_B(0)*_dist_antenna;
            _mocap_vec.y = subscribed_ext_2d_position.ext_pos_y/1000 + x_B(1)*_dist_antenna;
            /* give the mocap topic the new values with depth transformation to bodycenter coordinates */
            _mocap_vec.z = water_depth + (-_dist_pressure_sensor)*x_B(2); // double check this one - should be correct for pressure sensor mounted in back
            if (n <30) {
                        PX4_INFO("After RF/Pressure Compensation:\t%.4f\t%.4f\t%.4f",
                            (double)_mocap_vec.x,
                            (double)_mocap_vec.y,
                            (double)_mocap_vec.z);

                      }
        }else{
            /*Without coordinate transformation*/
            _mocap_vec.timestamp = hrt_absolute_time();
            _mocap_vec.timestamp_received = hrt_absolute_time();
            _mocap_vec.x = subscribed_ext_2d_position.ext_pos_x/1000;
            _mocap_vec.y = subscribed_ext_2d_position.ext_pos_y/1000;
            _mocap_vec.z = water_depth;
            if (n <30) {
                        PX4_INFO("Est Position without Compensation:\t%.4f\t%.4f\t%.4f",
                            (double)_mocap_vec.x,
                            (double)_mocap_vec.y,
                            (double)_mocap_vec.z);

                      }
        }
        _debug_vect.z = subscribed_ext_2d_position.ext_pos_x/1000;
        //orb_publish(ORB_ID(debug_vect), debug_vect_pub, &_debug_vect);

        parameters_update();
    }
    /* copy mocap data for publishing */
    orb_copy(ORB_ID(att_pos_mocap), att_pos_mocap_int, &_mocap_vec);

}

void Read_EXT_Data::task_main()
{
 _vehicle_local_position = orb_subscribe(ORB_ID(vehicle_local_position));
 _pressure_raw = orb_subscribe(ORB_ID(pressure));
 _vehicle_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
subscribed_ext_2d_position_int = orb_subscribe(ORB_ID(ext_2d_position));
att_pos_mocap_int = orb_subscribe(ORB_ID(att_pos_mocap));



	while (!_task_should_exit) {

           usleep(20000);
           ext_update_poll();

          // debug_vect_pub = orb_advertise(ORB_ID(debug_vect), &_debug_vect);
          // orb_publish(ORB_ID(debug_vect), debug_vect_pub, &_debug_vect);


           att_pos_mocap_pub = orb_advertise(ORB_ID(att_pos_mocap), &_mocap_vec);
           orb_publish(ORB_ID(att_pos_mocap), att_pos_mocap_pub, &_mocap_vec);

     }

}

void Read_EXT_Data::task_main_trampoline(int argc, char *argv[])
{
   ext_pose::g_control->task_main();
}


int Read_EXT_Data::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("ext_pose",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&Read_EXT_Data::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }
    PX4_INFO("ext pose module started!");
    return OK;
}


int ext_pose_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: ext_pose {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (ext_pose::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        ext_pose::g_control = new Read_EXT_Data;

        if (ext_pose::g_control == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != ext_pose::g_control->start()) {
            delete ext_pose::g_control;
           ext_pose::g_control = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (ext_pose::g_control == nullptr) {
            warnx("not running");
            return 1;
        }

        delete ext_pose::g_control;
        ext_pose::g_control = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (ext_pose::g_control) {
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

