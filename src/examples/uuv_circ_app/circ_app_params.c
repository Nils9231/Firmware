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
 * @file circ_app_params.c
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
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_NUM, 2.0f);

/**
 * Vehicle Order in Row
 * 
 * to adjust angle failure vor same distance on circle
 *
 * @min 1.0
 * @max 5.0
 * @increment 1.0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_ORDER, 2.0f);
 
/**
 * Proportional Gain of Midllepoint error
 * 
 * influence of midllepoint error on target yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_KCIRCP, 1.0f);

/**
 * differential Gain of Midllepoint error
 * 
 * influence of midllepoint error on target yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @reboot_required true 0
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_KCIRCD, 1.0f);


/**
 * Proportional Gain of Desired middlepoint
 * 
 * Influence on Middlepoint error of desired Middlepoint
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_KDES, 0.0f);

/**
 * Bool for aequidistance
 * 
 * T
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_TAEQ, 0.0f);

/**
 * starts while
 * 
 * Influence on Middlepoint error of desired Middlepoint
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_SR, 0.0f);

/**
 * constant yaw_rate control input
 * 
 * To set a better circular motion
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_YCONST, 0.0f);

/**
 * Proportional Speed Gain
 * 
 * To manage same distance on circle
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_KSP, 0.5f);

/**
 * Proportional Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_KPY, 1.0f);

/**
 * Proportional Gain Pitch_rate PID
 * 
 * Pitch_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
//PARAM_DEFINE_FLOAT(UUV_CIRC_KPF, 2.0f);

/**
 * Proportional Gain Roll_rate PID
 * 
 * Roll_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
//PARAM_DEFINE_FLOAT(UUV_CIRC_KPRO, 5.0f);

/**
 * Integrator Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_KIY, 0.0f);

/**
 * Integrator Gain Pitch_rate PID
 * 
 * Pitch_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
//PARAM_DEFINE_FLOAT(UUV_CIRC_KIF, 2.0f);

/**
 * Integrator Gain Roll_rate PID
 * 
 * Roll_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
//PARAM_DEFINE_FLOAT(UUV_CIRC_KIRO, 3.0f); 

/**
 * Differentiator Gain Yaw_rate PID
 * 
 * Yaw_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_KDY, 0.0f); 

/**
 * Differentiator Gain Pitch_rate PID
 * 
 * Pitch_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
//PARAM_DEFINE_FLOAT(UUV_CIRC_KDF, 4.0f); 

/**
 * Differentiator Gain Roll_rate PID
 * 
 * Roll_rate
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
//PARAM_DEFINE_FLOAT(UUV_CIRC_KDRO, 4.0f); 

/**
 * X-Value of desirec Cicle Middlepoint
 * 
 * cdes
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_CDES_X, 0.0f); 

/**
 * Y-Value of desirec Cicle Middlepoint
 * 
 * cdes
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_CDES_Y, 0.0f); 

/**
 * Z-Value of desirec Cicle Middlepoint
 * 
 * cdes
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_CDES_Z, 0.0f); 
 
/**
 * Desired angular Velocity
 * 
 * ome0
 *
 * @min 0.0
 * @increment 0.01
 * @group UUV_CIRC
 */
PARAM_DEFINE_FLOAT(UUV_CIRC_OME, 0.25f); 



