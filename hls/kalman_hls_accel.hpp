/*
 * Copyright 2019 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _XF_KALMANFILTER_CONFIG_H_
#define _XF_KALMANFILTER_CONFIG_H_

#include "common/xf_common.hpp"
#include "common/xf_utility.hpp"

#define KF_N 8        // States
#define KF_M 4        // Measurements
#define KF_C 0        // Control
#define KF_MTU 2      // Mutlipliers to use during time update
#define KF_MMU 2      // Multipliers to use during measurement update
#define XF_USE_URAM 0 // Use Ultra RAM
#define KF_EKF 0      // Use extended KF

#include "video/xf_kalmanfilter.hpp"

// Set the pixel depth:
#define TYPE XF_32FC1
#define PTR_WIDTH 32

// Set the optimization type:
#define NPC1 XF_NPPC1

// Control flags
#define INIT_EN 1
#define TIMEUPDATE_EN 2
#define MEASUPDATE_EN 4
#define XOUT_EN_TU 8
#define UDOUT_EN_TU 16
#define XOUT_EN_MU 32
#define UDOUT_EN_MU 64
#define EKF_MEM_OPT 128

#endif //_XF_KALMANFILTER_CONFIG_H_
