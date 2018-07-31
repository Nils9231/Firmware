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
#include <parameters/param.h>

/**
 * @file path_contr_params.c
 * Parameters for path controller for the Hippocampus.
 *
 * @author Nils Rottmann <Nils.Rottmann@tuhh.de>
 */


/**
 * K_P proportional position gain
 *
 * Factor for the position error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_PX, 2.0f); //1
PARAM_DEFINE_FLOAT(PC_K_PY, 2.0f); //1
PARAM_DEFINE_FLOAT(PC_K_PZ, 1.0f); //1

/**
 * K_V derivative position gain
 *
 * Factor for the velocity error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_VX, 1.5f); //1
PARAM_DEFINE_FLOAT(PC_K_VY, 1.5f); //1
PARAM_DEFINE_FLOAT(PC_K_VZ, 1.0f); //1

/**
 * K_P proportional angel gain
 *
 * Factor for the angular error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_RX, 1.0f); //1
PARAM_DEFINE_FLOAT(PC_K_RY, 2.0f); //1
PARAM_DEFINE_FLOAT(PC_K_RZ, 2.0f); //1

/**
 * K_W derivative angel gain
 *
 * Factor for the angel velocity error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_WX, 1.0f); //1
PARAM_DEFINE_FLOAT(PC_K_WY, 1.5f); //1
PARAM_DEFINE_FLOAT(PC_K_WZ, 1.5f);  //1

/**
 * Mass of the Hippocampus
 *
 */
PARAM_DEFINE_FLOAT(PC_m, 1.47f);

/**
 * Added mass X-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_X_du, 1.11f);

/**
 * Added mass Y-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_Y_du, 2.8f);

/**
 * Added mass Z-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_Z_dw, 2.8f);

/**
 * Damping X-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_X_u, 5.39f);

/**
 * Damping Y-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_Y_v, 17.36f);

/**
 * Damping Z-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_Z_w, 17.36f);

/**
 * Force scaling constant, should be the same as given in the simulation
 *
 */
PARAM_DEFINE_FLOAT(PC_K_F, 3.0f);

/**
 * Moment scaling constant, should be the same as given in the simulation
 *
 */
PARAM_DEFINE_FLOAT(PC_K_M, 0.02f);

/**
 * lifting arm for the Forces to generate Moments in pitch and yaw direction
 *
 */
PARAM_DEFINE_FLOAT(PC_L, 0.0481f);

/**
 * Operating Grade
 *
 */
PARAM_DEFINE_FLOAT(UUV_OG_THRUST, 0.0f);
PARAM_DEFINE_FLOAT(UUV_OG_YAW, 0.0f);
PARAM_DEFINE_FLOAT(UUV_OG_THRUST_C, 0.0f);
PARAM_DEFINE_FLOAT(UUV_OG_YAW_C, 0.0f);
PARAM_DEFINE_FLOAT(UUV_OG_ROLL, 0.0f);
PARAM_DEFINE_FLOAT(UUV_OG_PITCH, 0.0f);

/**
 * Set Pitch and Roll only = 1 with yaw and thrust =0
 *
 */
PARAM_DEFINE_FLOAT(UUV_PI_RO_ONLY, 0.0f);

/**
 * Desired Angle Roll
 *
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_DES, 0.0f);

/**
 * Desired Angle Pitch
 *
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_DES, 0.0f); //1.5

/**
 * Desired Angle Yaw
 *
 */
PARAM_DEFINE_FLOAT(UUV_YAW_DES, 0.0f);

/**
 * Actuator_control vector scale: Scale if Scale = 1 no scale if SCALE = 0
 *
 */
PARAM_DEFINE_FLOAT(UUV_SCALE, 0.0f);


/**
 * Scaling factor to avoid saturation
 *
 */
PARAM_DEFINE_FLOAT(UUV_SCALE_SAT, 0.0f);

/**
 * Mix-matrix from simulation =0; Mix-matrix from real system =1;
 *
 */
PARAM_DEFINE_FLOAT(UUV_MIX, 0.0f);
PARAM_DEFINE_FLOAT(UUV_DEPTH, 0.0f);
PARAM_DEFINE_FLOAT(UUV_DEPTH_P, 0.0f);
PARAM_DEFINE_FLOAT(UUV_DEPTH_I, 0.0f);
PARAM_DEFINE_FLOAT(UUV_DEPTH_D, 0.0f);
PARAM_DEFINE_FLOAT(UUV_SMO_RHO, 0.0f);
PARAM_DEFINE_FLOAT(UUV_SMO_TAU, 0.0f);
PARAM_DEFINE_FLOAT(UUV_SMO_PHI, 0.0f);

PARAM_DEFINE_FLOAT(UUV_ROLL_P, 0.0f);
PARAM_DEFINE_FLOAT(UUV_ROLL_D, 0.0f);

PARAM_DEFINE_FLOAT(UUV_PITCH_P, 0.0f);
PARAM_DEFINE_FLOAT(UUV_PITCH_D, 0.0f);
PARAM_DEFINE_FLOAT(UUV_YAW_P, 0.0f);
PARAM_DEFINE_FLOAT(UUV_YAW_D, 0.0f);

PARAM_DEFINE_FLOAT(UUV_THRUST_SP, 0.0f);
PARAM_DEFINE_FLOAT(UUV_YAW_RATE_SP, 0.0f);

PARAM_DEFINE_FLOAT(UUV_ROLL_SP_DEG, 0.0f);
PARAM_DEFINE_FLOAT(UUV_PITCH_SP_DEG, 0.0f);

PARAM_DEFINE_FLOAT(UUV_WGHT_PITCH1, 0.0f);
PARAM_DEFINE_FLOAT(UUV_WGHT_DEPTH1, 0.0f);
PARAM_DEFINE_FLOAT(UUV_WGHT_PITCH2, 0.0f);
PARAM_DEFINE_FLOAT(UUV_WGHT_DEPTH2, 0.0f);
PARAM_DEFINE_FLOAT(UUV_WS_CONTROL, 0.0f);
PARAM_DEFINE_FLOAT(UUV_NO_BACK, 0.0f);
PARAM_DEFINE_FLOAT(UUV_PITCH_CONT, 0.0f);
PARAM_DEFINE_FLOAT(UUV_PITCH_DES_L, 0.0f);
