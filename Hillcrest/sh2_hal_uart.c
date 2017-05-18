/**
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

/**
 * @file sh2_hal_uart.c
 * @author David Wheeler
 * @date 20 Apr 2017
 * @brief SH2 HAL Implementation for BNO080, via UART on STM32F411re Nucleo board
 *        with FreeRTOS.
 */


#include <string.h>

#include "sh2_hal.h"
#include "sh2_err.h"
#include "dbg.h"


#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "usart.h"

// Timing parameters for DFU
#define DFU_BOOT_DELAY (50)          // [mS]
#define RESET_DELAY    (10)          // [mS]

#define DFU_BPS (115200)             // 115200 bps for DFU
#define SH2_BPS (3000000)            // 3Mbps for BNO080 UART-SHTP

#define MAX_EVENTS (16)
#define SHTP_HEADER_LEN (4)

#define RSTN_GPIO_PORT GPIOB
#define RSTN_GPIO_PIN  GPIO_PIN_4

#define BOOTN_GPIO_PORT GPIOB
#define BOOTN_GPIO_PIN  GPIO_PIN_5

#define CSN_GPIO_PORT SH_CSN_GPIO_Port // from STM32Cube pin config via mxconstants.h
#define CSN_GPIO_PIN  SH_CSN_Pin

#define WAKEN_GPIO_PORT SH_WAKEN_GPIO_Port // from STM32Cube pin config via mxconstants.h
#define WAKEN_GPIO_PIN  SH_WAKEN_Pin

#define TIMESTAMP_0 (0)

#define PROTOCOL_CONTROL (0)
#define PROTOCOL_SHTP (1)

// ----------------------------------------------------------------------------------
// Private types
// ----------------------------------------------------------------------------------
typedef enum {
    OUTSIDE_FRAME,   // Waiting for start of frame
    INSIDE_FRAME,    // Inside frame until end of frame
    ESCAPED,         // Inside frame, after escape char
} RxState_t;

#define RFC1662_FLAG (0x7e)
#define RFC1662_ESCAPE (0x7d)

// ----------------------------------------------------------------------------------
// Private data
// ----------------------------------------------------------------------------------

// DMA stream for USART1 Rx.
DMA_HandleTypeDef hdma_usart1_rx;

// USART1 handle
UART_HandleTypeDef huart1;

bool sh2HalResetting = false;

// receive support
static bool rxReceivedData = false;
static uint32_t rxIndex = 0;
static uint8_t rxBuffer[256];
static volatile uint32_t rxChars;
static volatile uint32_t rxEvents;
static RxState_t rxState;

// Receive frame
#define MAX_FRAME_LEN 1024
static uint8_t rxFrame[MAX_FRAME_LEN];
static uint32_t rxFrameLen = 0;
static bool rxFrameOverflowed = false;
static uint32_t rxOverflowFrames = 0;
static uint32_t rxUnkProtocol = 0;

// Transmit support
static SemaphoreHandle_t txFrameBufSem;
#define MAX_TX_FRAME_LEN (SH2_HAL_MAX_TRANSFER*2)
static uint8_t txFrame[MAX_TX_FRAME_LEN];
static uint32_t txFrameLen;
static uint16_t txBsnAvail = 0;

// Buffer Status Query message
static const uint8_t bsq[] = {RFC1662_FLAG, PROTOCOL_CONTROL, RFC1662_FLAG};

// UART access

// HAL Tasks
static SemaphoreHandle_t txTaskHasWork;
static osThreadId txTaskHandle;

static SemaphoreHandle_t rxTaskHasWork;
static osThreadId rxTaskHandle;
static TimerHandle_t rxTimer;

static uint32_t rxTimestamp_uS;

typedef struct {
    bool dfuMode;
    void (*rstn)(bool);
    void (*bootn)(bool);

    // client callback support
    sh2_rxCallback_t *onRx;
    void *onRxCookie;

    SemaphoreHandle_t blockSem;
    
} Sh2Hal_t;
static Sh2Hal_t sh2Hal;

// ----------------------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------------------
static int usart_init(uint32_t baudrate);
static void rxTask(const void *params);
static void txTask(const void *params);
static void rstn0(bool state);
static void bootn0(bool state);
static void sh2HalRxCplt(UART_HandleTypeDef *huart);
static void sh2HalTxCplt(UART_HandleTypeDef *huart);
static void onRxTimer(TimerHandle_t t);
static void rxResetFrame(void);
static uint32_t rfc1662Encode(uint8_t *buf, uint32_t bufLen,
                              uint8_t protocolId,
                              uint8_t *pData, uint32_t dataLen);

// ----------------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------------
// Initialize SH-2 HAL subsystem
void sh2_hal_init(void) 
{
    // Initialize SH2 device
    memset(&sh2Hal, 0, sizeof(sh2Hal));

    // Semaphore to block clients with block/unblock API
    sh2Hal.blockSem = xSemaphoreCreateBinary();

    sh2Hal.rstn = rstn0;
    sh2Hal.bootn = bootn0;

    sh2Hal.rstn(false);  // Hold in reset
    sh2Hal.bootn(true);  // SH-2, not DFU

    // Create timer for polling for rx characters.
    rxTimer = xTimerCreate("rxTimer", 1, pdTRUE, 0, onRxTimer);

    // Create signal to wake rx task.
    rxTaskHasWork = xSemaphoreCreateBinary();
    
    // Initialize tx task resources
    txTaskHasWork = xSemaphoreCreateBinary();
    txFrameBufSem = xSemaphoreCreateBinary();
    txFrameLen = 0;
    xSemaphoreGive(txFrameBufSem);
    
    // register for rx, tx callbacks
    usartRegister(&huart1, sh2HalRxCplt, sh2HalTxCplt);

    rxResetFrame();
    rxState = OUTSIDE_FRAME;
    
    // Create rx task
    osThreadDef(rxThreadDef, rxTask, osPriorityNormal, 1, 2048);
    rxTaskHandle = osThreadCreate(osThread(rxThreadDef), NULL);
    if (rxTaskHandle == NULL) {
        printf("Failed to create SH-2 HAL rx task.\n");
    }

    // Create tx task
    osThreadDef(txThreadDef, txTask, osPriorityNormal, 1, 2048);
    txTaskHandle = osThreadCreate(osThread(txThreadDef), NULL);
    if (txTaskHandle == NULL) {
        printf("Failed to create SH-2 HAL tx task.\n");
    }
}

// Reset an SH-2 module (into DFU mode, if flag is true)
// The onRx callback function is registered with the HAL at the same time.
int sh2_hal_reset(bool dfuMode,
                  sh2_rxCallback_t *onRx,
                  void *cookie)
{
    // Store params
    sh2Hal.onRxCookie = cookie;
    sh2Hal.onRx = onRx;
    sh2Hal.dfuMode = dfuMode;

    // Assert reset
    sh2Hal.rstn(0);
    sh2HalResetting = true;
            
    xSemaphoreGive(rxTaskHasWork);
    xSemaphoreGive(txTaskHasWork);
        
    // Set BOOTN according to dfuMode
    sh2Hal.bootn(dfuMode ? 0 : 1);

    // Wait for reset to take effect
    vTaskDelay(RESET_DELAY);

    // === During this interval, txTask, rxTask process reset event ===

    // Initialize USART peripheral
    if (sh2Hal.dfuMode) {
        usart_init(DFU_BPS);
    }
    else {
        usart_init(SH2_BPS);
    }

    sh2HalResetting = false;
    
    // Enable INTN interrupt
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
       
    // Deassert reset
    sh2Hal.rstn(1);

    // If reset into DFU mode, wait until bootloader should be ready
    if (sh2Hal.dfuMode) {
        vTaskDelay(DFU_BOOT_DELAY);
    }

    return SH2_OK;
}

// Send data to SH-2
int sh2_hal_tx(uint8_t *pData, uint32_t len)
{
    
    if (!sh2Hal.dfuMode) {
        // Get semaphore for tx frame buffer
        // (If an earlier tx is still in progress, this will block.)
        xSemaphoreTake(txFrameBufSem, portMAX_DELAY);

        // Write contents of frame buffer
        uint32_t encLen = rfc1662Encode(txFrame, sizeof(txFrame), PROTOCOL_SHTP, pData, len);
        if (encLen == 0) {
            // Encoding didn't work, fail
            xSemaphoreGive(txFrameBufSem);
            return SH2_ERR_BAD_PARAM;
        }
        txFrameLen = encLen;
    }
    else {
        // DFU transmit
        memcpy(txFrame, pData, len);
        txFrameLen = len;
    }

    // Signal tx task that a frame is ready
    xSemaphoreGive(txTaskHasWork);
    
    return SH2_OK;
}

// Initiate a read of <len> bytes from SH-2
// This is a blocking read, pData will contain read data on return
// if return value was SH2_OK.
int sh2_hal_rx(uint8_t* pData, uint32_t len)
{
    // Clear received frame
    rxFrameLen = 0;

    // Wait for chars to arrive
    while (rxFrameLen < len) {
        // Bail out if reset occurs
        if (sh2HalResetting) {
            return SH2_ERR_IO;
        }
        
        vTaskDelay(1);
    }

    // Copy into pData
    memcpy(pData, rxFrame, len);

    return SH2_OK;
}

int sh2_hal_block(void)
{
    xSemaphoreTake(sh2Hal.blockSem, portMAX_DELAY);

    return SH2_OK;
}

int sh2_hal_unblock(void)
{
    xSemaphoreGive(sh2Hal.blockSem);

    return SH2_OK;
}

// ----------------------------------------------------------------------------------
// Callbacks for ISR, UART Operations
// ----------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    rxTimestamp_uS = xTaskGetTickCount()*1000;
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


// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    sh2HalRxCplt(huart);
}

static void sh2HalRxCplt(UART_HandleTypeDef *huart)
{
    BaseType_t woken = pdFALSE;
    
    rxReceivedData = true;
    xSemaphoreGiveFromISR(rxTaskHasWork, &woken);
    
    portYIELD_FROM_ISR(woken);
}

// 1ms timer, to check for received characters.
static void onRxTimer(TimerHandle_t t)
{
    rxReceivedData = true;
    xSemaphoreGive(rxTaskHasWork);
}

static void sh2HalTxCplt(UART_HandleTypeDef *huart)
{
    // TODO-DW
}

static int usart_init(uint32_t baudrate)
{
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);

#if 1
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
        Error_Handler();
    }

    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);
#endif
        
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
        return SH2_ERR_IO;
    }

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  
    return SH2_OK;
}

static void rxResetFrame(void)
{
    rxFrameLen = 0;
    rxFrameOverflowed = false;
}

static void rxAddToFrame(uint8_t c)
{
    if (rxFrameLen < sizeof(rxFrame)-1) {
        rxFrame[rxFrameLen++] = c;
    }
    else {
        rxFrameOverflowed = true;
    }
}

static void rxBsn(uint16_t available)
{
    // Record amount of space available
    txBsnAvail = available;

    // Wake TX task so it might send now.
    xSemaphoreGive(txTaskHasWork);
}

static void rxProcFrame(void)
{
    if (rxFrameOverflowed) {
        // Frame overflowed, discard it
        rxOverflowFrames++;
    }
    else if (rxFrame[0] == PROTOCOL_CONTROL) {
        // SHTP over UART Control Protocol
        if (rxFrameLen == 3) {
            // Received Buffer Status Notification
            rxBsn((rxFrame[2]<<8) + rxFrame[1]);
        }
    }
    else if (rxFrame[0] == PROTOCOL_SHTP) {
        // SHTP Payload, deliver it to client
        sh2Hal.onRx(sh2Hal.onRxCookie, rxFrame+1, rxFrameLen-1, rxTimestamp_uS);
    }
    else {
        // Unknown protocol?
        rxUnkProtocol++;
    }
}

static void rxCharShtp(uint8_t c)
{
    // Use state machine to build up chars into frames for delivery.
    switch (rxState) {
        case OUTSIDE_FRAME:
            // Look for start of frame
            if (c == RFC1662_FLAG) {
                // Init frame in progress
                rxResetFrame();
                rxState = INSIDE_FRAME;
            }
            break;
        case INSIDE_FRAME:
            // Look for end of frame
            if (c == RFC1662_FLAG) {
                if (rxFrameLen > 0) {
                    // Frame is done
                    rxProcFrame();
                    rxState = OUTSIDE_FRAME;
                }
                else {
                    // Treat second consec flag as another start flag.
                    rxResetFrame();
                    rxState = INSIDE_FRAME;
                }
            }
            else if (c == RFC1662_ESCAPE) {
                // Go to escaped state so next char can be a flag or escape
                rxState = ESCAPED;
            }
            else {
                // Add the character to the frame in progress
                rxAddToFrame(c);
            }
            break;
        case ESCAPED:
            rxAddToFrame(c);
            rxState = INSIDE_FRAME;
            break;
        default:
            // Bad state.  Recover by resetting to outside frame state
            rxState = OUTSIDE_FRAME;
            break;
    }
}

static void rxCharDfu(uint8_t c)
{
    rxAddToFrame(c);
}

static void rxTask(const void *params)
{
    // start timer to poll for data received by DMA
    xTimerStart(rxTimer, 0);

    rxReceivedData = false;
    
    // start receive data flow
    rxState = OUTSIDE_FRAME;
    HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));

    while (1) {
        // Wait until something happens
        xSemaphoreTake(rxTaskHasWork, portMAX_DELAY);

        // If resetting, do housekeeping and wait until it's time to run again.
        if (sh2HalResetting) {
            // Stop receive processing
            HAL_UART_DMAPause(&huart1);

            // Wait for reset period to end
            while (sh2HalResetting) {
                vTaskDelay(1);
            }

            // Restart receive process
            /// rxIndex = 0;
            rxResetFrame();
            rxState = OUTSIDE_FRAME;
            rxReceivedData = false;
            HAL_UART_DMAResume(&huart1);
            int status = HAL_UART_Receive_DMA(&huart1, rxBuffer, sizeof(rxBuffer));
        }

        if (rxReceivedData) {
            rxReceivedData = false;
            
            // Look at where DMA will write next char, that's where we stop.
            uint32_t stopPoint = sizeof(rxBuffer)-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

            // Process chars, incrementing rxIndex, until it reaches stop point
            while (rxIndex != stopPoint) {
                if (!sh2Hal.dfuMode) {
                    rxCharShtp(rxBuffer[rxIndex]);
                }
                else {
                    rxCharDfu(rxBuffer[rxIndex]);
                }
                rxIndex = (rxIndex+1) & sizeof(rxBuffer)-1;
            }
        }
    }
}

void sendBuffer(const uint8_t *pData, uint32_t len)
{
    // Send data 1 byte at a time with 1ms delays.
    // (SHTP spec calls for 100uS delay minimum)
    for (unsigned n = 0; n < len; n++) {
        vTaskDelay(1);
        
        if (sh2HalResetting) {
            // Stop in the middle of buffer if reset occurs.
            return;
        }
        
        HAL_UART_Transmit_IT(&huart1, (uint8_t *)(pData+n), 1);
    }
}

static void txTask(const void *params)
{
    while (1) {
        // Wait until something happens
        xSemaphoreTake(txTaskHasWork, portMAX_DELAY);

        if (sh2HalResetting) {
            // Perform RESET actions
            rxFrameLen = 0;
            rxFrameOverflowed = false;
            txBsnAvail = 0;

            // Wait until reset completes
            while (sh2HalResetting) {
                vTaskDelay(1);
            }
        }

        // Either a new frame was posted or a new BSN was received.
        if (txFrameLen > 0) {
            // Stuff needs to be sent.

            if (!sh2Hal.dfuMode) {
                if (txFrameLen <= txBsnAvail) {
                    // BNO080 can take it now.
                    // Sending a frame erases what we knew from last BSN
                    txBsnAvail = 0;

                    // Send the frame
                    sendBuffer(txFrame, txFrameLen);

                    // release txFrame so it can be refilled.
                    txFrameLen = 0;
                    xSemaphoreGive(txFrameBufSem);
                }
                else {
                    // BNO080 can't take it yet, send a buffer status query
                    sendBuffer(bsq, sizeof(bsq));
                }
            }
            else {
                // DFU Mode transmit
                HAL_UART_Transmit(&huart1, txFrame, txFrameLen, HAL_MAX_DELAY);
            }
        }
    }
}

static void bootn0(bool state)
{
	HAL_GPIO_WritePin(BOOTN_GPIO_PORT, BOOTN_GPIO_PIN, 
	                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rstn0(bool state)
{
	HAL_GPIO_WritePin(RSTN_GPIO_PORT, RSTN_GPIO_PIN, 
	                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t rfc1662Encode(uint8_t *buf, uint32_t bufLen,
                              uint8_t protocolId,
                              uint8_t *pData, uint32_t dataLen)
{
    uint32_t encLen = 0;

    // Add start flag
    // (bufLen checks at each insert return error on overflow.)
    if (bufLen <= encLen) goto error;
    buf[encLen++] = RFC1662_FLAG;

    // Add protocol id
    if (bufLen <= encLen) goto error;
    buf[encLen++] = protocolId;
    
    // Copy pData to buf, escaping flags and escape chars in content
    for (uint32_t n = 0; n < dataLen; n++) {
        // If next byte needs to be escaped...
        if ((pData[n] == RFC1662_FLAG) || (pData[n] == RFC1662_ESCAPE)) {
            // If overflowed, return error
            if (bufLen <= encLen-1) goto error;

            // Add escaped data to buffer
            buf[encLen++] = RFC1662_ESCAPE;
            buf[encLen++] = pData[n];
        }
        else {
            // If overflowed, return error
            if (bufLen <= encLen) goto error;

            // Add data to buffer
            buf[encLen++] = pData[n];
        }
    }
    
    // Add stop flag
    if (bufLen <= encLen) goto error;
    buf[encLen++] = RFC1662_FLAG;
    
    // return the length of data in the buffer
    return encLen;
    
    // On error, return 0
error:
    return 0;
}

