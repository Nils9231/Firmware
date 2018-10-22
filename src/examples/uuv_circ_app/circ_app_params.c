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
 * @file path_controller_params.c
 * Parameters for path controller for the Hippocampus.
 *
 * @author Nils Timmermann <nils.timmermann@tuhh.de>
 */

/**
 * Number of vehicles
 *
 * 
 * 
 * @min 1
 * @max 5
 * @increment 1
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_NUM, 2f);

/**
 * Vehicle Order in Row
 * 
 * to adjust angle failure vor same distance on circle
 *
 * @min 1
 * @max 5
 * @increment 1
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Order, 2f);
 
/**
 * Proportional Gain of Midllepoint error
 * 
 * influence of midllepoint error on target yaw_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kcirc, 1f);


/**
 * Proportional Gain of Desired middlepoint
 * 
 * Influence on Middlepoint error of desired Middlepoint
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kdes, 0f);

/**
 * constant yaw_rate control input
 * 
 * To set a better circular motion
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Yconst, 0f);

/**
 * Proportional Speed Gain
 * 
 * To manage same distance on circle
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Ksp, 0.5f);

/**
 * Proportional Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kpy, 1f);

/**
 * Proportional Gain Pitch_rate PID
 * 
 * Pitch_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kpf, 2f);

/**
 * Proportional Gain Roll_rate PID
 * 
 * Roll_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kpro, 5f);

/**
 * Integrator Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kiy, 0f);

/**
 * Integrator Gain Pitch_rate PID
 * 
 * Pitch_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kif, 2f);

/**
 * Integrator Gain Roll_rate PID
 * 
 * Roll_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kiro, 3f); 

/**
 * Differentiator Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kdy, 0f); 

/**
 * Differentiator Gain Pitch_rate PID
 * 
 * Pitch_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kdf, 4f); 

/**
 * Differentiator Gain Roll_rate PID
 * 
 * Roll_rate
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_Kdro, 4f); 

/**
 * X-Value of desirec Cicle Middlepoint
 * 
 * cdes
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_cdes_x, 0f); 

/**
 * Y-Value of desirec Cicle Middlepoint
 * 
 * cdes
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_cdes_y, 0f); 

/**
 * Z-Value of desirec Cicle Middlepoint
 * 
 * cdes
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_cdes_z, 0f); 
 
/**
 * Desired angular Velocity
 * 
 * ome0
 *
 * @min 0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_ome0, 0.25f); 



