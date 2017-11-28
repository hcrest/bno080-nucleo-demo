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

/*
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

#define RSTN_GPIO_PORT GPIOB
#define RSTN_GPIO_PIN  GPIO_PIN_4

#define BOOTN_GPIO_PORT GPIOB
#define BOOTN_GPIO_PIN  GPIO_PIN_5

#define CSN_GPIO_PORT SH_CSN_GPIO_Port // from STM32Cube pin config via mxconstants.h
#define CSN_GPIO_PIN  SH_CSN_Pin

#define WAKEN_GPIO_PORT SH_WAKEN_GPIO_Port // from STM32Cube pin config via mxconstants.h
#define WAKEN_GPIO_PIN  SH_WAKEN_Pin

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

// Flag to coordinate reset operations between tasks.
bool sh2HalResetting = false;

// receive support
static bool rxReceivedData = false;  // tells rx task to process rx data
static uint8_t rxBuffer[256];        // receives UART data via DMA
static uint32_t rxIndex = 0;         // next index to read

// Receive frame
#define MAX_FRAME_LEN 1024
static uint8_t rxFrame[MAX_FRAME_LEN]; // receives RFC1662 decoded frame
static uint32_t rxFrameLen = 0;        // length of frame so far
static RxState_t rxState;              // RFC1662 decoder state
static bool rxFrameOverflowed = false; // Becomes true on overflow

// Transmit support
#define MAX_TX_FRAME_LEN (SH2_HAL_MAX_TRANSFER*2)
static uint8_t txFrame[MAX_TX_FRAME_LEN];// tx frame buffer
static uint32_t txFrameLen;              // length of frame in txFrame[]
static SemaphoreHandle_t txFrameBufSem;  // to serialize access to tx frame buf.
static uint16_t txBsnAvail = 0;          // Length of frame we're allowed to send

// Buffer Status Query message
static const uint8_t bsq[] = {RFC1662_FLAG, PROTOCOL_CONTROL, RFC1662_FLAG};

// Statistics (for debugging)
static uint32_t rxOverflowFrames = 0;  // Counter of overflow incidents
static uint32_t rxUnkProtocol = 0;     // Counter of unknown protocol ids

// HAL Tasks
static SemaphoreHandle_t txTaskHasWork; // signals tx task to handle event
static osThreadId txTaskHandle;         // tx task handle

static SemaphoreHandle_t rxTaskHasWork; // signals rx task to handle event
static osThreadId rxTaskHandle;         // rx task handle
static TimerHandle_t rxTimer;           // timer to check DMA periodically

static uint32_t rxTimestamp_uS;         // timestamp of INTN event

// mode flag from last reset operation
static bool dfuMode = false;

// client callback support
static sh2_rxCallback_t *onRx;
static void *onRxCookie;

// Semaphore supporting sh2_hal_block/unblock operations
static SemaphoreHandle_t blockSem;
    
// ----------------------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------------------
static int setupUsart(uint32_t baudrate);
static void rxTask(const void *params);
static void txTask(const void *params);
static void rstn(bool state);
static void bootn(bool state);
static void ps0_waken(bool state);
static void onRxCplt(UART_HandleTypeDef *huart);
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
    dfuMode = false;
    rstn(false);  // Hold in reset
    bootn(!dfuMode);  // SH-2, not DFU

    // register for rx, tx callbacks
    usartRegisterHandlers(&huart1, onRxCplt, 0);

    // Semaphore to block clients with block/unblock API
    blockSem = xSemaphoreCreateBinary();

    // Initialize rx task resources
    rxTimer = xTimerCreate("rxTimer", 1, pdTRUE, 0, onRxTimer);
    rxTaskHasWork = xSemaphoreCreateBinary();
    rxResetFrame();
    rxState = OUTSIDE_FRAME;
    
    // Initialize tx task resources
    txFrameLen = 0;
    txTaskHasWork = xSemaphoreCreateBinary();
    txFrameBufSem = xSemaphoreCreateBinary();
    xSemaphoreGive(txFrameBufSem);
    
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
    
    // Enable INTN interrupt
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Reset an SH-2 module (into DFU mode, if flag is true)
// The onRx callback function is registered with the HAL at the same time.
int sh2_hal_reset(bool _dfuMode,
                  sh2_rxCallback_t *_onRx,
                  void *cookie)
{
    // Store params
    onRxCookie = cookie;
    onRx = _onRx;
    dfuMode = _dfuMode;

    // Assert reset
    rstn(0);

    // Set BOOTN according to dfuMode
    bootn(!dfuMode);

    // To boot in SHTP-UART mode, must have PS1=1, PS0=0.
    // PS1 is set via jumper.
    // PS0 will be 0 if PS0 jumper is 0 OR (PS1 jumper is 1 AND WAKEN sig is 0)
    // So we set WAKEN signal to 0 just in case PS1 jumper is in 1 position.
    ps0_waken(false);

    // Notify tasks that HAL is resetting
    sh2HalResetting = true;
    xSemaphoreGive(rxTaskHasWork);
    xSemaphoreGive(txTaskHasWork);
        
    // Wait for reset to take effect
    vTaskDelay(RESET_DELAY);

    // === During this interval, txTask, rxTask process reset event ===

    // Initialize USART peripheral
    if (dfuMode) {
        setupUsart(DFU_BPS);
    }
    else {
        setupUsart(SH2_BPS);
    }

    sh2HalResetting = false;
    
    // Deassert reset
    rstn(1);

    // If reset into DFU mode, wait until bootloader should be ready
    if (dfuMode) {
        vTaskDelay(DFU_BOOT_DELAY);
    }

    return SH2_OK;
}

// Send data to SH-2
int sh2_hal_tx(uint8_t *pData, uint32_t len)
{
    // Get semaphore for tx frame buffer
    // (If an earlier tx is still in progress, this will block.)
    xSemaphoreTake(txFrameBufSem, portMAX_DELAY);

    if (!dfuMode) {
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

    // Signal tx task that a frame is ready to send
    xSemaphoreGive(txTaskHasWork);
    
    return SH2_OK;
}

// Initiate a read of <len> bytes from SH-2
// This is a blocking read, pData will contain read data on return
// if return value was SH2_OK.
int sh2_hal_rx(uint8_t* pData, uint32_t len)
{
    // NOTE: sh2_hal_rx is only called in DFU mode.
    if (!dfuMode) {
        return SH2_ERR_BAD_PARAM;
    }
    
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
    xSemaphoreTake(blockSem, portMAX_DELAY);

    return SH2_OK;
}

int sh2_hal_unblock(void)
{
    xSemaphoreGive(blockSem);

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

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    onRxCplt(huart);
}

// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

static void onRxCplt(UART_HandleTypeDef *huart)
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

static int setupUsart(uint32_t baudrate)
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
        return SH2_ERR_IO;
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
        onRx(onRxCookie, rxFrame+1, rxFrameLen-1, rxTimestamp_uS);
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
            rxAddToFrame(c ^ 0x20);
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
                vTaskDelay(0);
            }

            // Restart receive process
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
                if (!dfuMode) {
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

static void delay100uS(void)
{
    volatile uint32_t n;

    // This wastes 100uS of this task's time while allowing other tasks to run.
    // If other tasks do run, the delay will be greater than 100uS.  That's OK.
    // count of 28 : 57uS
    // count of 56 : 109uS
    for (n = 0; n < 56; n++) {
        vTaskDelay(0);
    }
}

void sendBuffer(const uint8_t *pData, uint32_t len)
{
    // Send data 1 byte at a time with 1ms delays.
    // (SHTP spec calls for 100uS delay minimum)
    for (unsigned n = 0; n < len; n++) {
        delay100uS();
        
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
            rxResetFrame();
            txBsnAvail = 0;

            // Clear tx frame buffer
            txFrameLen = 0;
            xSemaphoreGive(txFrameBufSem);
            
            // Wait until reset completes
            while (sh2HalResetting) {
                vTaskDelay(0);
            }
        }

        // Either a new frame was posted or a new BSN was received.
        if (txFrameLen > 0) {
            // Stuff needs to be sent.

            if (!dfuMode) {
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

                // Transmit done, release the semaphore.
                xSemaphoreGive(txFrameBufSem);
            }
        }
    }
}

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

static void ps0_waken(bool state)
{
	HAL_GPIO_WritePin(WAKEN_GPIO_PORT, WAKEN_GPIO_PIN, 
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
            buf[encLen++] = pData[n] ^ 0x20;
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

