/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PRINT_H
#define PRINT_H
/* Define to prevent recursive inclusion -------------------------------------*/

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include <stddef.h> 

void UART_config (size_t peripheral_clock, size_t UART_baudrate);
void print (const char* ptr); // UART debug print

#endif /* PRINT_H */