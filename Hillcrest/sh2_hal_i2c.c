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
 * @file sh2_hal_i2c.c
 * @author David Wheeler
 * @date 18 Nov 2016
 * @brief SH2 HAL Implementation for BNO080, via I2C on STM32F411re Nucleo board
 *        with FreeRTOS.
 */

#include <string.h>

#include "sh2_hal.h"
#include "shtp.h"
#include "sh2_err.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define DFU_BOOT_DELAY (200) // [mS]
#define RESET_DELAY    (10) // [mS]

#define MAX_EVENTS (16)
#define SHTP_HEADER_LEN (4)

#define ADDR_DFU_0 (0x28)
#define ADDR_DFU_1 (0x29)
#define ADDR_SH2_0 (0x4A)
#define ADDR_SH2_1 (0x4B)

#define RSTN_GPIO_PORT GPIOB
#define RSTN_GPIO_PIN  GPIO_PIN_4

#define BOOTN_GPIO_PORT GPIOB
#define BOOTN_GPIO_PIN  GPIO_PIN_5

// ----------------------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------------------
static void halTask(const void *params);
static void i2cReset(void);
static int i2cBlockingRx(unsigned addr, uint8_t* pData, unsigned len);
static int i2cBlockingTx(unsigned addr, uint8_t* pData, unsigned len);
static void rstn0(bool state);
static void bootn0(bool state);

// ----------------------------------------------------------------------------------
// Private data
// ----------------------------------------------------------------------------------

// I2C Bus access
static I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef *hi2c;
static SemaphoreHandle_t i2cMutex;
static SemaphoreHandle_t i2cBlockSem;
int i2cStatus;
bool i2cResetNeeded;

// HAL Queue and Task
static QueueHandle_t evtQueue = 0;
osThreadId halTaskHandle;

typedef struct {
    void (*rstn)(bool);
    void (*bootn)(bool);
    sh2_rxCallback_t *onRx;
    void *onRxCookie;
    uint16_t addr;
    uint8_t rxBuf[SH2_HAL_MAX_TRANSFER];
    uint16_t rxRemaining;
    SemaphoreHandle_t blockSem;
} Sh2Hal_t;
Sh2Hal_t sh2Hal;

typedef enum {
    EVT_INTN,
} EventId_t;

typedef struct {
    uint32_t t_ms;
    EventId_t id;
} Event_t;

// ----------------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------------
// Initialize SH-2 HAL subsystem
void sh2_hal_init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        // This should not happen, debug it.
        while (1);
    }
    
    // Store reference to I2C peripheral
    hi2c = &hi2c1;

    // Need to init I2C peripheral before first use.
    i2cResetNeeded = true;
    
    // Initialize SH2 HAL data structure
    memset(&sh2Hal, 0, sizeof(sh2Hal));

    // Semaphore to block clients with block/unblock API
    sh2Hal.blockSem = xSemaphoreCreateBinary();
    
    sh2Hal.rstn = rstn0;
    sh2Hal.bootn = bootn0;

    // Put SH2 device in reset
    sh2Hal.rstn(false);  // Hold in reset
    sh2Hal.bootn(true);  // SH-2, not DFU

    // init mutex for i2c bus
    i2cMutex = xSemaphoreCreateMutex();
    i2cBlockSem = xSemaphoreCreateBinary();

    // Create queue to pass events from ISRs to task context.
    evtQueue = xQueueCreate(MAX_EVENTS, sizeof(Event_t));
    if (evtQueue == NULL) {
        printf("The queue could not be created.\n");
    }

    // Create task
    osThreadDef(halThreadDef, halTask, osPriorityNormal, 0, 256);
    halTaskHandle = osThreadCreate(osThread(halThreadDef), NULL);
    if (halTaskHandle == NULL) {
        printf("Failed to create SH-2 HAL task.\n");
    }
}

// Reset an SH-2 module (into DFU mode, if flag is true)
// The onRx callback function is registered with the HAL at the same time.
int sh2_hal_reset(bool dfuMode,
                  sh2_rxCallback_t *onRx,
                  void *cookie)
{
    // Get exclusive access to i2c bus (blocking until we do.)
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Store params for later reference
    sh2Hal.onRxCookie = cookie;
    sh2Hal.onRx = onRx;

    // Set addr to use in this mode
    sh2Hal.addr = dfuMode ? ADDR_DFU_0<<1 : ADDR_SH2_0<<1;
    
    // Assert reset
    sh2Hal.rstn(0);
    
    // Set BOOTN according to dfuMode
    sh2Hal.bootn(dfuMode ? 0 : 1);

    // Wait for reset to take effect
    vTaskDelay(RESET_DELAY);

    // Enable INTN interrupt
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
       
    // Deassert reset
    sh2Hal.rstn(1);

    // If reset into DFU mode, wait until bootloader should be ready
    if (dfuMode) {
        vTaskDelay(DFU_BOOT_DELAY);
    }

    // Will need to reset the i2c peripheral after this.
    i2cResetNeeded = true;
    
    // Give up ownership of i2c bus.
    xSemaphoreGive(i2cMutex);

    return SH2_OK;
}

// Send data to SH-2
int sh2_hal_tx(uint8_t *pData, uint32_t len)
{
    // Do nothing if len is zero
    if (len == 0) {
        return SH2_OK;
    }

    // Do tx, and return when done
    return i2cBlockingTx(sh2Hal.addr, pData, len);
}

// Initiate a read of <len> bytes from SH-2
// This is a blocking read, pData will contain read data on return
// if return value was SH2_OK.
int sh2_hal_rx(uint8_t* pData, uint32_t len)
{
    // Do nothing if len is zero
    if (len == 0) {
        return SH2_OK;
    }

    // do rx and return when done
    return i2cBlockingRx(sh2Hal.addr, pData, len);
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
// Callbacks for ISR, I2C Operations
// ----------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    BaseType_t woken= pdFALSE;
    Event_t event;
        
    event.t_ms = xTaskGetTickCount();
    event.id = EVT_INTN;
    
    xQueueSendFromISR(evtQueue, &event, &woken);
    portEND_SWITCHING_ISR(woken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    BaseType_t woken= pdFALSE;

    // Set status from this operation
    i2cStatus = SH2_OK;

    // Unblock the caller
    xSemaphoreGiveFromISR(i2cBlockSem, &woken);
    
    portEND_SWITCHING_ISR(woken);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    BaseType_t woken= pdFALSE;

    // Set status from this operation
    i2cStatus = SH2_OK;

    // Unblock the caller
    xSemaphoreGiveFromISR(i2cBlockSem, &woken);
    
    portEND_SWITCHING_ISR(woken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c)
{
    BaseType_t woken= pdFALSE;

    // Set status from this operation
    i2cStatus = SH2_ERR_IO;

    // Unblock the caller
    xSemaphoreGiveFromISR(i2cBlockSem, &woken);
    
    portEND_SWITCHING_ISR(woken);
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
* @brief This function handles I2C1 error interrupt.
*/
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

static void halTask(const void *params)
{
    Event_t event;
    unsigned readLen = 0;
    unsigned cargoLen = 0;

    while (1) {
        // Block until there is work to do
        xQueueReceive(evtQueue, &event, portMAX_DELAY);

        // Handle the event
        switch (event.id) {
            case EVT_INTN:
                // If no RX callback registered, don't bother trying to read
                if (sh2Hal.onRx != 0) {
                    // Compute read length
                    readLen = sh2Hal.rxRemaining;
                    if (readLen < SHTP_HEADER_LEN) {
                        // always read at least the SHTP header
                        readLen = SHTP_HEADER_LEN;
                    }
                    if (readLen > SH2_HAL_MAX_TRANSFER) {
                        // limit reads to transfer size
                        readLen = SH2_HAL_MAX_TRANSFER;
                    }

                    // Read i2c
                    i2cBlockingRx(sh2Hal.addr, sh2Hal.rxBuf, readLen);

                    // Get total cargo length from SHTP header
                    cargoLen = ((sh2Hal.rxBuf[1] << 8) + (sh2Hal.rxBuf[0])) & (~0x8000);
                
                    // Re-Evaluate rxRemaining
                    if (cargoLen > readLen) {
                        // More to read.
                        sh2Hal.rxRemaining = (cargoLen - readLen) + SHTP_HEADER_LEN;
                    }
                    else {
                        // All done, next read should be header only.
                        sh2Hal.rxRemaining = 0;
                    }

                    // Deliver via onRx callback
                    sh2Hal.onRx(sh2Hal.onRxCookie, sh2Hal.rxBuf, readLen, event.t_ms * 1000);
                }

                break;
            default:
                // Unknown event type.  Ignore.
                break;
        }
    }
}

// Perform a blocking i2c read
static int i2cBlockingRx(unsigned addr, uint8_t* pData, unsigned len)
{
    int status = SH2_OK;
    
    // Get device mutex
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Reset bus, if necc.
    if (i2cResetNeeded) {
        i2cReset();
    }
    
    // Call I2C API rx
    int rc = HAL_I2C_Master_Receive_IT(hi2c, addr, pData, len);

    if (rc == 0) {
        // Block on results
        xSemaphoreTake(i2cBlockSem, portMAX_DELAY);
    
        // Set return status
        status = i2cStatus;
    }
    else {
        // I2C operation failed
        status = SH2_ERR_IO;
    }
    
    // Release device mutex
    xSemaphoreGive(i2cMutex);

    return status;
}

static int i2cBlockingTx(unsigned addr, uint8_t* pData, unsigned len)
{
    int status = SH2_OK;
    
    // Get device mutex
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Reset bus, if necc.
    if (i2cResetNeeded) {
        i2cReset();
    }
    
    // Call I2C API rx
    int rc = HAL_I2C_Master_Transmit_IT(hi2c, addr, pData, len);

    if (rc == 0) {
        // Block on results
        xSemaphoreTake(i2cBlockSem, portMAX_DELAY);
    
        // Set return status
        status = i2cStatus;
    }
    else {
        // I2C operation failed
        status = SH2_ERR_IO;
    }
    
    // Release device mutex
    xSemaphoreGive(i2cMutex);

    return status;
}

// DeInit and Init I2C Peripheral.
// (This recovery step is necessary after resets of the device)
static void i2cReset(void)
{
    HAL_I2C_DeInit(hi2c);
        
    hi2c->Instance = I2C1;
    hi2c->Init.ClockSpeed = 400000;
    hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    
    HAL_I2C_Init(hi2c);

    i2cResetNeeded = false;
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
