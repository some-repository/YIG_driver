#include "print.h"

void UART_config (uint32_t peripheral_clock, uint32_t UART_baudrate)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_USART1);
    // PA15 (TX1)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_15, LL_GPIO_AF_7);

    LL_USART_SetDataWidth (USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetParity (USART1, LL_USART_PARITY_NONE);
    LL_USART_SetOverSampling (USART1, LL_USART_OVERSAMPLING_16);
    LL_USART_SetStopBitsLength (USART1, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate (USART1, peripheral_clock, LL_USART_OVERSAMPLING_16, UART_baudrate);
    LL_USART_SetTransferDirection (USART1, LL_USART_DIRECTION_TX);
    LL_USART_SetHWFlowCtrl (USART1, LL_USART_HWCONTROL_NONE);
    LL_USART_ConfigAsyncMode (USART1);
    LL_USART_Enable (USART1);
}

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