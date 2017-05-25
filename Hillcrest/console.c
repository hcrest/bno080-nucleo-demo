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
 * usart2 console
 * Supports standard i/o over VCOM USB interface on Nucleo F401/F411 boards.
 */

#include "console.h"

#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include <FreeRTOS.h>
#include <semphr.h>

#include "usart.h"

#define CONSOLE_BUFLEN (128)

// ------------------------------------------------------------------------
// Private state variables

// The UART used by the console
UART_HandleTypeDef huart2;
UART_HandleTypeDef *console_huart = 0;

// Transmit state
volatile bool txActive;
volatile bool txBlocked;
SemaphoreHandle_t txBlockSem;
SemaphoreHandle_t txMutex;

// Which buffer owned by process, other owned by interrupt.
unsigned txPhase = 0;

// Double buffers for transmit
uint8_t txBuffer[2][CONSOLE_BUFLEN];
volatile unsigned txBufLen[2];

SemaphoreHandle_t rxBlockSem;
SemaphoreHandle_t rxMutex;
bool rxBlocked;
bool rxActive;
uint8_t rxChar;
uint8_t rxBuffer[CONSOLE_BUFLEN];
unsigned rxNextIn;
unsigned rxNextOut;
unsigned rxDrops;

// ------------------------------------------------------------------------
// Forward declarations

void startTx(void);
void startTxIsr(void);
static void consoleRxCplt(UART_HandleTypeDef *huart);
static void consoleTxCplt(UART_HandleTypeDef *huart);

// ------------------------------------------------------------------------
// Public API

void console_init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        // Should not happen, debug!
        while(1);
    }
    
    console_huart = &huart2;

    usartRegisterHandlers(console_huart, consoleRxCplt, consoleTxCplt);

	txActive = false;
	txBlocked = false;
	txBlockSem = xSemaphoreCreateBinary();
	txMutex = xSemaphoreCreateMutex();
	txPhase = 0;
	txBufLen[0] = 0;
	txBufLen[1] = 0;

    // Receive support
	rxBlocked = false;
	rxBlockSem = xSemaphoreCreateBinary();
	rxMutex = xSemaphoreCreateMutex();
	rxNextIn = 0;
	rxNextOut = 0;  // rxBuffer empty when rxNextIn == rxNextOut
	rxDrops = 0;
	rxActive = false;

    // Enable interrupts now that we're ready.
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

size_t __read(int Handle, unsigned char * Buf, size_t BufSize)
{
	size_t copied = 0;

	// This function only works for stdin
	if (Handle != 0) {
		return -1;
	}
	
	// While there's room and data to copy
	while (copied < BufSize) {
		Buf[copied++] = getchar();
	}

	return copied;
}

int getchar(void)
{
	static unsigned count = 0;
	int c = -1;
        
        count++;
	
	// Acquire mutex to prevent tasks from stomping each other.
	xSemaphoreTake(rxMutex, portMAX_DELAY);

	if (!rxActive) {
		// Start receiving.
		rxActive = true;
		HAL_UART_Receive_IT(console_huart, &rxChar, 1);
	}
	
	// Disable USART2 interrupts to maintain consistency
	HAL_NVIC_DisableIRQ(USART2_IRQn);

	if (rxNextIn == rxNextOut) {
		// buffer empty, block until something arrives
		rxBlocked = true;

		// enable interrupts again to permit receive processing
		HAL_NVIC_EnableIRQ(USART2_IRQn);

		// Wait for stuff
		xSemaphoreTake(rxBlockSem, portMAX_DELAY);
	} else {
		// Re-enable interrupts
		HAL_NVIC_EnableIRQ(USART2_IRQn);
	}

	// Either way, UART2 interrupts are enabled now

	// Take a character from head of queue
	c = rxBuffer[rxNextOut];
	unsigned outIndex = rxNextOut;
	outIndex++;
	if (outIndex >= sizeof(rxBuffer)) {
		outIndex = 0;
	}
	rxNextOut = outIndex;

	xSemaphoreGive(rxMutex);
        
	// translate CR to LF
	if (c == '\r') {
		c = '\n';
	}
        
	// echo
	putchar(c);

	return c;
}

size_t __write(int Handle, const unsigned char * Buf, size_t Bufsize)
{
	size_t n;
	int rc;

	// This function only works for stdout, stderr
	if (!((Handle == 1) || (Handle == 2))) {
		return -1;
	}

	for (n = 0; n < Bufsize; n++) {
		rc = putchar(Buf[n]);
		if (rc < 0) break;
	}

	return n;
}

int putchar(int c)
{
	// expand LF to CR-LF
	if (c == '\n') {
		int rc = putchar('\r');
		if (rc < 0) {
			return rc;
		}
	}
	
	// Acquire mutex to prevent tasks from stomping each other.
	xSemaphoreTake(txMutex, portMAX_DELAY);
	
	// Disable USART2 interrupt while manipulating tx buffers
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	
	// If current buffer is full, block until it's empty
	if (txBufLen[txPhase] == CONSOLE_BUFLEN) {
		txBlocked = true;
		
		// Re-enable USART2 interrupt while blocking
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		
		// Block on semaphore until ISR frees up space.
		xSemaphoreTake(txBlockSem, portMAX_DELAY);
		
		// Disable USART2 interrupt again while blocking
		HAL_NVIC_DisableIRQ(USART2_IRQn);
	}
	
	// Put this character into current buffer
	txBuffer[txPhase][txBufLen[txPhase]] = c;
	txBufLen[txPhase]++;

	// If tx is inactive, start it.
	if (!txActive) {
		// Start a new transmission
		startTx();
	} 
	
	// Re-enable USART2 interrupts now
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	
	// Allow other tasks to transmit again.
	xSemaphoreGive(txMutex);

	return c;
}

// ------------------------------------------------------------------------
// Private utility functions

static void startTx(void)
{
	unsigned isrBuf = txPhase;

	// Swap buffers and clear new tx buffer
	txPhase = txPhase ? 0 : 1;
	txBufLen[txPhase] = 0;
	
	// Start transmission of current buffer
	txActive = true;
	HAL_UART_Transmit_IT(console_huart, txBuffer[isrBuf], txBufLen[isrBuf]);
}

static void startTxIsr(void)
{
	BaseType_t woken = pdFALSE;

	startTx();

	if (txBlocked) {
		txBlocked = false;
		xSemaphoreGiveFromISR(txBlockSem, &woken);
	}
	
	portYIELD_FROM_ISR(woken);
}

void consoleTxCplt(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		// One transmission is complete.
		if (txBufLen[txPhase] != 0) {
			// Start the next transmission
			startTxIsr();
		}
		else {
			// Go to inactive state
			txActive = false;
		}
	}
}

static void consoleRxCplt(UART_HandleTypeDef *huart)
{
	BaseType_t woken = pdFALSE;
	
	// rxChar now has the latest input.

	unsigned nextInsertPoint = rxNextIn + 1;
	if (nextInsertPoint >= sizeof(rxBuffer)) {
		nextInsertPoint = 0;
	}

	if (nextInsertPoint == rxNextOut) {
		// circular queue is full, drop this
		rxDrops++;
	}
	else {
		// Put the character into circular queue
		rxBuffer[rxNextIn] = rxChar;
		rxNextIn = nextInsertPoint;
	}

	// If the receiving process needs to wake up, do it.
	if (rxBlocked) {
		rxBlocked = false;
		xSemaphoreGiveFromISR(rxBlockSem, &woken);
	}

	HAL_UART_Receive_IT(huart, &rxChar, 1);

	portYIELD_FROM_ISR(woken);
}

