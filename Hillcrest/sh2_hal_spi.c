/**
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
 * @file sh2_hal_spi.c
 * @author David Wheeler
 * @date 2 Dec 2016
 * @brief SH2 HAL Implementation for BNO080, via SPI on STM32F411re Nucleo board
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

// Timing parameters for DFU
#define DFU_BOOT_DELAY (50)          // [mS]
#define RESET_DELAY    (10)          // [mS]
#define DFU_CS_DEASSERT_DELAY_RX (0) // [mS]
#define DFU_CS_DEASSERT_DELAY_TX (5) // [mS]
#define DFU_CS_TIMING_US (20)        // [uS]
#define DFU_BYTE_TIMING_US (28)      // [uS]

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

// ----------------------------------------------------------------------------------
// Private data
// ----------------------------------------------------------------------------------

typedef enum {
    TRANSFER_IDLE = 0,    // SPI device not in use
    TRANSFER_DATA,        // Transferring bulk of data (DFU transfer or second phase SHTP)
    TRANSFER_HDR,         // Transferring first two bytes of SHTP
} TransferPhase_t;

// SPI Bus access
static SPI_HandleTypeDef hspi1;
static SPI_HandleTypeDef *hspi;
static SemaphoreHandle_t spiMutex;
static int spiOpStatus;
static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER];
static const uint8_t* spiTxData;
static uint8_t* spiRxData;
static uint16_t spiTransferLen;
static TransferPhase_t transferPhase;

// HAL Queue and Task
static QueueHandle_t evtQueue = 0;
osThreadId halTaskHandle;


typedef struct {
    bool dfuMode;
    void (*rstn)(bool);
    void (*bootn)(bool);
    void (*csn)(bool);
    void (*waken)(bool);
    sh2_rxCallback_t *onRx;
    void *onRxCookie;

    // Rx resources
    uint8_t rxBuf[SH2_HAL_MAX_TRANSFER];
    uint16_t rxLen;

    // Tx resources
    SemaphoreHandle_t txMutex;
    uint8_t txBuf[SH2_HAL_MAX_TRANSFER];
    uint16_t txLen;

    uint32_t pending_t_uS;
    uint32_t t_uS;
    
    SemaphoreHandle_t blockSem;
    
    // State machine state
    enum {
        DEV_IDLE,
        DEV_IN_PROG,
        DEV_NEW_INTN,
    } state;
} Dev_t;
Dev_t dev;

typedef enum {
    EVT_INTN,
    EVT_OP_CPLT,
    EVT_OP_ERR,
} EventId_t;

typedef struct {
    EventId_t id;
    uint32_t t_uS;
} Event_t;

// ----------------------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------------------
static void halTask(const void *params);
static void rstn0(bool state);
static void bootn0(bool state);
static void csn0(bool state);
static void waken0(bool state);
static void spiReset(bool dfuMode);

static int tx_dfu(uint8_t* pData, uint32_t len);
static int tx_shtp(uint8_t* pData, uint32_t len);
static int rx_dfu(uint8_t* pData, uint32_t len);

static void delayUs(uint32_t count);


// ----------------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------------
// Initialize SH-2 HAL subsystem
void sh2_hal_init()
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 2.6MHz
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        // This should not have happened, debug it.
        while(1);
    }
    
    // Store reference to SPI peripheral
    hspi = &hspi1;

    spiReset(false);

    // Initialize SH2 device
    memset(&dev, 0, sizeof(dev));

    // Semaphore to protect transmit state
    dev.txMutex = xSemaphoreCreateBinary();
    xSemaphoreGive(dev.txMutex);

    // Semaphore to block clients with block/unblock API
    dev.blockSem = xSemaphoreCreateBinary();

    dev.rstn = rstn0;
    dev.bootn = bootn0;
    dev.csn = csn0;
    dev.waken = waken0;

    dev.rstn(false);  // Hold in reset
    dev.bootn(true);  // SH-2, not DFU
    dev.csn(true);    // deassert CSN
    dev.waken(true);  // deassert WAKEN.

    // init mutex for spi bus
    spiMutex = xSemaphoreCreateBinary();
    xSemaphoreGive(spiMutex);

    // Create queue to pass events from ISRs to task context.
    evtQueue = xQueueCreate(MAX_EVENTS, sizeof(Event_t));
    if (evtQueue == NULL) {
        printf("The queue could not be created.\n");
    }

    // Create task
    osThreadDef(halThreadDef, halTask, osPriorityNormal, 1, 1024);
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
    // Get exclusive access to SPI bus (blocking until we do.)
    xSemaphoreTake(spiMutex, portMAX_DELAY);

    // Store params for later reference
    dev.dfuMode = dfuMode;
    dev.onRxCookie = cookie;
    dev.onRx = onRx;

    // Wait a bit before asserting reset.
    // (Because this may be a reset after a DFU and that process needs
    // an extra few ms to store data in flash before the device is
    // actually reset.)
    vTaskDelay(RESET_DELAY); 
       
    // Assert reset
    dev.rstn(0);

    // Deassert CSN in case it was asserted
    dev.csn(1);
    
    // Set BOOTN according to dfuMode
    dev.bootn(dfuMode ? 0 : 1);

    // set PS0 (WAKEN) to support booting into SPI mode.
    dev.waken(1);
    
    // Reset SPI parameters
    spiReset(dfuMode);

    // Wait for reset to take effect
    vTaskDelay(RESET_DELAY); 
       
    // Enable INTN interrupt
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
       
    // Deassert reset
    dev.rstn(1);

    // If reset into DFU mode, wait until bootloader should be ready
    if (dfuMode) {
        vTaskDelay(DFU_BOOT_DELAY);
    }

    // Give up ownership of SPI bus.
    xSemaphoreGive(spiMutex);

    return SH2_OK;
}

// Send data to SH-2
int sh2_hal_tx(uint8_t *pData, uint32_t len)
{
    // Do nothing if len is zero
    if (len == 0) {
        return SH2_OK;
    }

    if (dev.dfuMode) {
        return tx_dfu(pData, len);
    }
    else {
        return tx_shtp(pData, len);
    }
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

    if (dev.dfuMode) {
        return rx_dfu(pData, len);
    }
    else {
        // sh2_hal_rx API isn't used in non-DFU mode.
        return SH2_ERR;
    }
}

int sh2_hal_block(void)
{
    xSemaphoreTake(dev.blockSem, portMAX_DELAY);

    return SH2_OK;
}

int sh2_hal_unblock(void)
{
    xSemaphoreGive(dev.blockSem);

    return SH2_OK;
}

// ----------------------------------------------------------------------------------
// Callbacks for ISR, SPI Operations
// ----------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t n)
{
    BaseType_t woken= pdFALSE;
    Event_t event;
        
    event.t_uS = xTaskGetTickCount()*1000;
    event.id = EVT_INTN;
    
    xQueueSendFromISR(evtQueue, &event, &woken);
    portEND_SWITCHING_ISR(woken);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
    BaseType_t woken= pdFALSE;
    bool opFinished = false;
    Event_t event;

    dbgPulse(2);
    
    // What to do next depends on transfer phase
    if (transferPhase == TRANSFER_HDR) {
        uint16_t txLen = (spiTxData[0] + (spiTxData[1] << 8) & ~0x8000);
        uint16_t rxLen = (spiRxData[0] + (spiRxData[1] << 8) & ~0x8000);
        if (rxLen == 0x7FFF) {
            // 0x7FFF is an invalid length
            rxLen = 0;
        }
        
        uint16_t len = (txLen > rxLen) ? txLen : rxLen;
        if (len > SH2_HAL_MAX_TRANSFER) {
            len = SH2_HAL_MAX_TRANSFER;
        }

        if (len == 0) {
            // Nothing left to transfer!
            spiOpStatus = SH2_OK;
            opFinished = true;
        }
        else {
            // Start data phase of tranfer
            transferPhase = TRANSFER_DATA;
            spiTransferLen = len;
    
            dbgPulse(5);
            int rc = HAL_SPI_TransmitReceive_IT(hspi, (uint8_t*)(spiTxData+2), (spiRxData+2), len-2);
            if (rc != 0) {
                // Signal IO Error to HAL task
                spiOpStatus = SH2_ERR_IO;
                opFinished = true;
            }
        }
    }
    else if (transferPhase == TRANSFER_DATA) {
        spiOpStatus = SH2_OK;
        opFinished = true;
    }

    // If operation finished for any reason, unblock the caller
    if (opFinished) {
        transferPhase = TRANSFER_IDLE;
        
        event.id = EVT_OP_CPLT;
        event.t_uS = 99;    // not used

        xQueueSendFromISR(evtQueue, &event, &woken);
    }

    portEND_SWITCHING_ISR(woken);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi)
{
    BaseType_t woken= pdFALSE;
    Event_t event;

    dbgPulse(3);
    
    // transfer is over
    transferPhase = TRANSFER_IDLE;

    // Set status from this operation
    spiOpStatus = SH2_ERR_IO;

    event.id = EVT_OP_ERR;
    event.t_uS = 99;    // not used
    
    xQueueSendFromISR(evtQueue, &event, &woken);
    
    portEND_SWITCHING_ISR(woken);
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi1);
}

// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

static void takeBus(void)
{
    // get bus mutex
    xSemaphoreTake(spiMutex, portMAX_DELAY);
}

static void relBus(void)
{
    // Release bus
    xSemaphoreGive(spiMutex);
}

static void deliverRx(uint32_t t_uS)
{
    // Deliver results via onRx callback
    if (dev.onRx != 0) {
        if (dev.rxLen) {
            dev.onRx(dev.onRxCookie, dev.rxBuf, dev.rxLen, t_uS);
        }
    }
}

static void endOpShtp(void)
{
    // record rx len
    dev.rxLen = spiTransferLen;
    
    // Release transmit mutex if it's held.
    if (dev.txLen) {
        dev.txLen = 0;
        dbgClr();
        xSemaphoreGive(dev.txMutex);
    }

    // deassert CSN
    dev.csn(true);

    // Release the bus
    relBus();
}

static int startOpShtp(void)
{
    int retval = 0;
    
    // Set up operation on bus
    takeBus();
                    
    // assert CSN
    dev.csn(false);
    
    // Read into device's rxBuf
    spiRxData = dev.rxBuf;
                    
    // If there is stuff to transmit, deassert WAKE and do it now.
    spiTxData = txZeros;
    if (dev.txLen) {
        dev.waken(true);
        spiTxData = dev.txBuf;
    }

    // initiate (Header phase of) transfer
    transferPhase = TRANSFER_HDR;
    spiTransferLen = 2;
    dbgPulse(5);
    int rc = HAL_SPI_TransmitReceive_IT(hspi, (uint8_t*)spiTxData, spiRxData, 2);
    if (rc != 0) {
        // Failed to start!  Abort!
        endOpShtp();
        retval = -1;
    }

    return retval;
}

static void halTask(const void *params)
{
    Event_t event;
    static volatile uint32_t trap = 0;
    static volatile uint32_t oops = 0;
    int rc;

    while (1) {
        // Block until there is work to do
        xQueueReceive(evtQueue, &event, portMAX_DELAY);

        // Handle the event
        switch (event.id) {
            case EVT_INTN:
                if (dev.dfuMode) {
                    // Ignore INTN in DFU mode
                }
                else {
                    if (dev.state == DEV_IDLE) {
                        // Start a new operation and go to IN-PROGRESS state
                        dev.state = DEV_IN_PROG;
                        dev.t_uS = event.t_uS;
                        rc = startOpShtp();
                        if (rc) {
                            // failure to start
                            dev.state = DEV_IDLE;
                        }
                    }
                    else {
                        // An operation is still in progress, go to NEW-INTN state
                        dev.pending_t_uS = event.t_uS;
                        dev.state = DEV_NEW_INTN;
                    }
                }
                break;
            case EVT_OP_CPLT:
                if (dev.dfuMode) {
                    // Ignore this event.
                    // By design, this shouldn't happen.  DFU mode doesn't use interrupt
                    // mode SPI API.
                }
                else {
                    // Post-op for operation that just completed
                    endOpShtp();

                    // Deliver received content
                    deliverRx(dev.t_uS);

                    // If a new INTN was signalled, start the next op
                    if (dev.state == DEV_NEW_INTN) {
                        // start next op
                        dev.t_uS = dev.pending_t_uS;
                        dev.state = DEV_IN_PROG;
                        rc = startOpShtp();
                        if (rc) {
                            // failure to start
                            dev.state = DEV_IDLE;
                        }
                    }
                    else {
                        // no operation in progress now.
                        dev.state = DEV_IDLE;
                    }
                }
                break;
            case EVT_OP_ERR:
                if (dev.dfuMode) {
                    // Ignore this event.
                    // By design, this shouldn't happen.  DFU mode doesn't use interrupt
                    // mode SPI API.
                }
                else {
                    endOpShtp();

                    // If a new INTN was signalled, start the next op
                    if (dev.state == DEV_NEW_INTN) {
                        // start next op
                        dev.t_uS = dev.pending_t_uS;
                        dev.state = DEV_IN_PROG;
                        rc = startOpShtp();
                        if (rc) {
                            // failure to start
                            dev.state = DEV_IDLE;
                        }
                    }
                    else {
                        // no operation in progress now.
                        dev.state = DEV_IDLE;
                    }
                }
                break;
            default:
                // Unknown event type.  Ignore.
                break;
        }
    }
}

// DeInit and Init SPI Peripheral.
static void spiReset(bool dfuMode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    
    HAL_SPI_DeInit(hspi);

    // Common parameters
    hspi->Instance = SPI1;
    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.CRCPolynomial = 10;

    // Differences between DFU and App
    if (dfuMode) {
        hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 84MHz / 128 -> 0.65MHz
    }
    else {
        hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  // 84MHz / 64 -> 1.3MHz
        // hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 84MHz / 128 -> 0.65MHz
    }
    
    HAL_SPI_Init(hspi);
    
    // We need to establish SCLK in proper initial state.
    // Do one SPI operation with reset asserted and no CS asserted to get clock sorted.
    uint8_t dummyTx[1];
    uint8_t dummyRx[1];
    
    memset(dummyTx, 0xAA, sizeof(dummyTx));
        
    dbgPulse(5);
    HAL_SPI_TransmitReceive(hspi, dummyTx, dummyRx, sizeof(dummyTx), 1);
}           

static int tx_dfu(uint8_t* pData, uint32_t len)
{
    int status = SH2_OK;

    takeBus();

    // assert CSN
    dev.csn(false);

    delayUs(DFU_CS_TIMING_US);
    
    // Set up Tx, Rx bufs
    spiTxData = pData;
    spiRxData = dev.rxBuf;

    // We will just use a simple one-phase transfer for DFU
    transferPhase = TRANSFER_DATA;
    
    // initiate transfers.  Do them one byte at a time because BNO needs some
    // time between bytes.
    int rc = 0;
    for (int n = 0; n < len; n++) {
        dbgPulse(5);
        rc = HAL_SPI_Transmit(hspi, (uint8_t*)spiTxData+n, 1, 1);
        if (rc != 0) {
            break;
        }
        delayUs(DFU_BYTE_TIMING_US);
    }
    spiTransferLen = len;

    if (rc == 0) {
        // Set return status
        dev.rxLen = spiTransferLen;
        status = spiOpStatus;
    }
    else {
        // SPI operation failed
        dev.rxLen = 0;
        status = SH2_ERR_IO;
    }

    // deassert CSN
    dev.csn(true);

    // Wait on each CSN assertion.  DFU Requires at least 5ms of deasserted time!
    vTaskDelay(DFU_CS_DEASSERT_DELAY_TX);

    relBus();
    
    return status;
}

static int tx_shtp(uint8_t* pData, uint32_t len)
{
    int status = SH2_OK;
    static volatile uint32_t wtf = 0;

    // Get semaphore on device
    wtf++;
    xSemaphoreTake(dev.txMutex, portMAX_DELAY);

    // Set up txLen and txBuf
    memset(dev.txBuf, 0, sizeof(dev.txBuf));
    memcpy(dev.txBuf, pData, len);
    
    // Assert WAKE
    dbgSet();
    dev.waken(false);

    // Finally, set len, which triggers tx processing in HAL task
    dev.txLen = len;
    
    // Transmission will take place after INTN is processed.
    
    return status;
}

static int rx_dfu(uint8_t* pData, uint32_t len)
{
    int status = SH2_OK;

    takeBus();

    // Wait on each CSN assertion.  DFU Requires at least 5ms of deasserted time!
    vTaskDelay(DFU_CS_DEASSERT_DELAY_RX);

    // assert CSN
    dev.csn(false);
    delayUs(DFU_CS_TIMING_US);
                    
    // Set up Tx, Rx bufs
    spiTxData = txZeros;
    spiRxData = pData;

    // We will just use a simple one-phase transfer for DFU
    transferPhase = TRANSFER_DATA;
    
    // initiate transfer
    int rc = 0;
    spiTransferLen = len;
    for (int n = 0; n < len; n++) {
        dbgPulse(5);
        rc = HAL_SPI_Receive(hspi, spiRxData+n, 1, 1);
        if (rc != 0) {
            break;
        }
        delayUs(DFU_BYTE_TIMING_US);
    }

    if (rc == 0) {
        // Set return status
        dev.rxLen = spiTransferLen;
        status = spiOpStatus;
    }
    else {
        // SPI operation failed
        dev.rxLen = 0;
        status = SH2_ERR_IO;
    }

    // deassert CSN
    dev.csn(true);

    // Wait after each CSN deassertion in DFU mode to ensure proper timing.
    vTaskDelay(DFU_CS_DEASSERT_DELAY_RX);

    relBus();
    
    return status;
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

static void csn0(bool state)
{
	HAL_GPIO_WritePin(CSN_GPIO_PORT, CSN_GPIO_PIN, 
	                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void waken0(bool state)
{
    HAL_GPIO_WritePin(WAKEN_GPIO_PORT, WAKEN_GPIO_PIN, 
	              state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#define DELAY_LOOP_1_US (8.4656)
#define DELAY_LOOP_OFFSET (-12.7)

static void delayUs(uint32_t count)
{
    float delayCount;
    volatile unsigned delayCounter;

    delayCount = DELAY_LOOP_1_US * count + DELAY_LOOP_OFFSET;
    if (delayCount < 1.0) return;
    
    delayCounter = (unsigned)delayCount;
    
    while (delayCounter > 0) delayCounter--;
}
