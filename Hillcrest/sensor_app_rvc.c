/*
 * Copyright 2017 Hillcrest Laboratories, Inc.
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
 * Demo App for Hillcrest BNO080, UART-RVC Mode input
 */


// Sensor Application
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include "sensor_app.h"
#include "usart.h"

#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#endif

#define FIX_Q(n, x) ((int32_t)(x * (float)(1 << n)))
const float scaleDegToRad = 3.14159265358 / 180.0;

#define RSTN_GPIO_PORT GPIOB
#define RSTN_GPIO_PIN  GPIO_PIN_4

#define BOOTN_GPIO_PORT GPIOB
#define BOOTN_GPIO_PIN  GPIO_PIN_5

#define WAKEN_GPIO_PORT SH_WAKEN_GPIO_Port // from STM32Cube pin config via mxconstants.h
#define WAKEN_GPIO_PIN  SH_WAKEN_Pin

#define MAX_FRAME_LEN (19)         // Length of UART-RVC Frame
#define RX_BUFFER_SIZE (64)        // Size of UART's DMA Buffer

#define RESET_DELAY    (10)          // [mS]

#define UART_RVC_BAUDRATE (115200) // UART-RVC fixed baud rate

// Offsets into rx frame
#define RVC_INDEX (2)
#define RVC_YAW_LSB (3)
#define RVC_YAW_MSB (4)
#define RVC_PITCH_LSB (5)
#define RVC_PITCH_MSB (6)
#define RVC_ROLL_LSB (7)
#define RVC_ROLL_MSB (8)
#define RVC_ACC_X_LSB (9)
#define RVC_ACC_X_MSB (10)
#define RVC_ACC_Y_LSB (11)
#define RVC_ACC_Y_MSB (12)
#define RVC_ACC_Z_LSB (13)
#define RVC_ACC_Z_MSB (14)

// --- Forward declarations -------------------------------------------

static void bootn(bool state);
static void rstn(bool state);
static void ps0(bool state);
static void setupUsart(uint32_t baudrate);
static void onRxCplt(UART_HandleTypeDef *huart);
static void onRxTimer(TimerHandle_t t);
static void rxChar(uint8_t c);

// --- Private data ---------------------------------------------------

// DMA stream for USART1 Rx.
DMA_HandleTypeDef hdma_usart1_rx;

// USART1 handle
UART_HandleTypeDef huart1;

// Rx Resources
static uint8_t rxBuffer[RX_BUFFER_SIZE]; // receives UART data via DMA
static uint32_t rxIndex = 0;             // next index to read

static TimerHandle_t rxTimer;           // timer to check DMA periodically
static SemaphoreHandle_t rxTaskHasWork; // signals rx task to handle event
static uint8_t rxFrame[MAX_FRAME_LEN];  // receives frame from BNO080
static uint32_t rxFrameLen = 0;         // length of frame so far

// --- Public methods -------------------------------------------------

void sh2_hal_init(void)
{
    // Dummy function.  UART-RVC Mode does not use an SH2 HAL.
}

void demoTaskStart(const void * params)
{
    printf("\n\nHillcrest SH-2 Demo.  RVC Mode.\n");
    
    // Initialize rx task resources
    rxTimer = xTimerCreate("rxTimer", 1, pdTRUE, 0, onRxTimer);
    rxTaskHasWork = xSemaphoreCreateBinary();
    rxFrameLen = 0;
    
    // register for rx, tx callbacks
    usartRegisterHandlers(&huart1, onRxCplt, 0);

    // Reset BNO080 (Deassert BOOTN)
    rstn(false);  // Hold in reset
    bootn(true);  // Don't enter DFU on boot

    // For RVC mode, boot with PS0=1, PS1=0.
    // So PS1 switch must be in 0 position, PS0 switch in 1 position
    // AND PS0/Wake signal from host must be high.
    ps0(true);    // Set PS0/Wake high

    // Start up UART
    setupUsart(UART_RVC_BAUDRATE);
    HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));

    // start timer to poll for data received by DMA
    xTimerStart(rxTimer, 0);

    // Wait for reset to take effect
    vTaskDelay(RESET_DELAY);
    
    // Release BNO080 from reset
    rstn(true);

    // Process incoming data
    while (1) {
        // Wait for data
        xSemaphoreTake(rxTaskHasWork, portMAX_DELAY);

        // Look at where DMA will write next char, that's where we stop reading.
        uint32_t stopPoint = sizeof(rxBuffer)-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        
        // Process new data
        while (rxIndex != stopPoint) {
            // process one input character
            rxChar(rxBuffer[rxIndex]);   
            rxIndex = (rxIndex+1) & (sizeof(rxBuffer)-1);
        }
    }
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    onRxCplt(huart);
}

// --- Private methods ----------------------------------------------

static void bootn(bool state)
{
	HAL_GPIO_WritePin(BOOTN_GPIO_PORT, BOOTN_GPIO_PIN, 
	                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rstn(bool state)
{
	HAL_GPIO_WritePin(RSTN_GPIO_PORT, RSTN_GPIO_PIN, 
	                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ps0(bool state)
{
	HAL_GPIO_WritePin(WAKEN_GPIO_PORT, WAKEN_GPIO_PIN, 
	                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void setupUsart(uint32_t baudrate)
{
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        // Error!
        return;
    }

    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);
        
    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        // Error!
        return;
    }

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

static void onRxCplt(UART_HandleTypeDef *huart)
{
    BaseType_t woken = pdFALSE;

    xSemaphoreGiveFromISR(rxTaskHasWork, &woken);
    
    portYIELD_FROM_ISR(woken);
}

static void onRxTimer(TimerHandle_t t)
{
    xSemaphoreGive(rxTaskHasWork);
}

static bool checksumOk()
{
    uint8_t check = 0;

    for (int n = 2; n < MAX_FRAME_LEN-1; n++) {
        check += rxFrame[n];
    }

    return (check == rxFrame[MAX_FRAME_LEN-1]);
}

static void processFrame(void)
{
    uint8_t index = rxFrame[RVC_INDEX];
    float yaw_deg =   0.01  * (int16_t)((rxFrame[RVC_YAW_MSB] << 8) + rxFrame[RVC_YAW_LSB]);
    float pitch_deg = 0.01  * (int16_t)((rxFrame[RVC_PITCH_MSB] << 8) + rxFrame[RVC_PITCH_LSB]);
    float roll_deg =  0.01  * (int16_t)((rxFrame[RVC_ROLL_MSB] << 8) + rxFrame[RVC_ROLL_LSB]);
    float acc_x_g =   0.001 * (int16_t)((rxFrame[RVC_ACC_X_MSB] << 8) + rxFrame[RVC_ACC_X_LSB]);
    float acc_y_g =   0.001 * (int16_t)((rxFrame[RVC_ACC_Y_MSB] << 8) + rxFrame[RVC_ACC_Y_LSB]);
    float acc_z_g =   0.001 * (int16_t)((rxFrame[RVC_ACC_Z_MSB] << 8) + rxFrame[RVC_ACC_Z_LSB]);

    printf("%3d : yaw:%0.2f pitch:%0.2f roll:%0.2f ax:%0.3f ay:%0.3f az:%0.3f\n",
           index, yaw_deg, pitch_deg, roll_deg, acc_x_g, acc_y_g, acc_z_g);
}

static void rxChar(uint8_t c)
{
    // If rx buffer is full, shift the data
    if (rxFrameLen == MAX_FRAME_LEN) {
        // Shift data
        for (int n = 0; n < MAX_FRAME_LEN-1; n++) {
            rxFrame[n] = rxFrame[n+1];
        }

        // Add newest char
        rxFrame[MAX_FRAME_LEN-1] = c;
    }
    else {
        // Add newest char
        rxFrame[rxFrameLen++] = c;
    }
    
    // If rx buffer is full, now, check framing
    if (rxFrameLen == MAX_FRAME_LEN) {
        if ((rxFrame[0] == 0xAA) && (rxFrame[1] == 0xAA)) {
            if (checksumOk()) {
                // Found a well formed frame, process frame
                processFrame();

                // Clear frame buffer
                rxFrameLen = 0;
            }
        }
    }
    
}



