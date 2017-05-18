// TODO-DW : Header, license

#include "usart.h"
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>

// ------------------------------------------------------------------------
// Private Type definitions

typedef struct {
    USART_TypeDef *Instance;
    UART_HandleTypeDef *huart;
    CpltCallback_t *rxCplt;
    CpltCallback_t *txCplt;
} UsartHandler_t;

#define NUM_USART (4)

// ------------------------------------------------------------------------
// Forward declarations

static void usartInit(void);

// ------------------------------------------------------------------------
// Private data
static UsartHandler_t handler[NUM_USART];
static bool usartInitialized = false;

// ------------------------------------------------------------------------
// Public API

void usartRegister(UART_HandleTypeDef *huart, CpltCallback_t *rxCplt, CpltCallback_t *txCplt)
{
    if (!usartInitialized) {
        usartInit();
    }

    // Clear any previous entry for this huart
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].Instance == huart->Instance) {
            handler[n].Instance = 0;
            handler[n].huart = 0;
            handler[n].rxCplt = 0;
            handler[n].txCplt = 0;
        }
    }

    // Find an open entry (or where this uart already registered)
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].Instance == 0) {
            handler[n].Instance = huart->Instance;
            handler[n].huart = huart;
            handler[n].rxCplt = rxCplt;
            handler[n].txCplt = txCplt;
            break;
        }
    }
}

void usartUnregister(UART_HandleTypeDef *huart, CpltCallback_t *rxCplt, CpltCallback_t *txCplt)
{
    if (!usartInitialized) {
        usartInit();
    }

    // Clear any previous entry for this huart
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].Instance == huart->Instance) {
            handler[n].Instance = 0;
            handler[n].huart = 0;
            handler[n].rxCplt = 0;
            handler[n].txCplt = 0;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].huart == huart) {
            handler[n].txCplt(huart);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int n = 0; n < NUM_USART; n++) {
        if (handler[n].huart == huart) {
            handler[n].rxCplt(huart);
        }
    }
}

// ------------------------------------------------------------------------
// Private functions

static void usartInit(void)
{
    // Initialize the array of registrations
    memset(handler, 0, sizeof(handler));

    usartInitialized = true;
}
