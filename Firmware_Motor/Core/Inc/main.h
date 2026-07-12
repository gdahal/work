#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Global declarations */
void Error_Handler(void);
uint8_t Hall_GetState(void);
void UART_SendMessage(const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
