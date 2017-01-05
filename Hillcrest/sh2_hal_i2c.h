/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file sh2_hal_demo.h
 * @author David Wheeler
 * @date 18 Nov 2016
 * @brief Hardware Adaptation Layer API for SensorHub-2 (and BNO080)
 */


#ifndef SH2_HAL_I2C_H
#define SH2_HAL_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "sh2_hal_impl.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

    // Initialize SH2 HAL Implementation
    void sh2_hal_init(I2C_HandleTypeDef* _hi2c);

#ifdef __cplusplus
}    // end of extern "C"
#endif

// #ifdef SH2_HAL_H
#endif
