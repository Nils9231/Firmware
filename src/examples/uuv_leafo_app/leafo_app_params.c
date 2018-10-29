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
 * @file leafo_app_params.c
 * Parameters for path controller for the Hippocampus.
 *
 * @author Nils Timmermann <nils.timmermann@tuhh.de>
 */

/**
 * Number of vehicles
 *
 * Number
 * 
 * @min 1.0
 * @max 5.0
 * @increment 1.0
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_NUM, 2.0f);

/**
 * Vehicle Order in Row
 * 
 * to adjust angle failure vor same distance on LEAFOle
 *
 * @min 1.0
 * @max 5.0
 * @increment 1.0
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_ORDER, 2.0f);
 
/**
 * Follow in Chain or all one leader
 * 
 * Follow in Chain or all one leader
 *
 * @min 0.0
 * @max 1.0
 * @increment 1.0
 * @reboot_required true 0
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_IC, 1.0f);


/**
 * Start While
 * 
 * Should Run
 *
 * @min 0.0
 * @max 1.0
 * @increment 1
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_SR, 0.0f);

/**
 * Proportional Gain SPeed
 * 
 * t
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_KSP, 0.5f);

/**
 * Proportional Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_KYP, 1.0f);

/**
 * Integrator Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_KYI, 0.0f);

/**
 * Differentiator Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_KYD, 0.0f); 
/**
 * Trajectory lead set
 * 
 * Traj
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_KT, 3.0f); 

/**
 * X-Value of desired Trajectory
 * 
 * T
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_T_X, 1.0f); 

/**
 * Y-Value of desired Trajectory
 * 
 * T
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_T_Y, 0.0f); 

/**
 * Z-Value of desired Trajectory
 * 
 * T
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_T_Z, 0.0f); 

/**
 * X-Value of desired Trajectory-Offset
 * 
 * Toff
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_TOFF_X, 0.0f); 

/**
 * Y-Value of desired Trajectory-Offset
 * 
 * Toff
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_TOFF_Y, 0.0f); 

/**
 * Z-Value of desired Trajectory-Offset
 * 
 * Toff
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_LEAFO
 */
PARAM_DEFINE_FLOAT(UUV_LEAFO_TOFF_Z, 0.0f); 


