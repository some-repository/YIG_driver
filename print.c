#include "print.h"

void print (const char* ptr) // UART debug print
{
    uint16_t i = 0;
    uint8_t byte = 0;

    while ((byte = (uint8_t) ptr [i]))
    {
        while (!LL_USART_IsActiveFlag_TXE (USART1));
        LL_USART_TransmitData8 (USART1, byte);
        i++;
    }
}