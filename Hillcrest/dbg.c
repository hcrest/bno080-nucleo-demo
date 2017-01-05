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

/*
 * GPIO control for debug support
 */


#include "dbg.h"

#include "stm32f4xx_hal.h"

#define DEBUG_GPIO_PORT GPIOB
#define DEBUG_GPIO_PIN  GPIO_PIN_3

void dbgInit()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*Configure GPIO pins : PB3, Debug */
  GPIO_InitStruct.Pin = DEBUG_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(DEBUG_GPIO_PORT, &GPIO_InitStruct);
}

void dbgPulse(unsigned count)
{
	for (unsigned n = 0; n < count; n++) {
		HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
	}
}

void dbgSet()
{
	HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_SET);
}

void dbgClr()
{
	HAL_GPIO_WritePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PIN_RESET);
}

     
