#include "stm32f411xe.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include <stddef.h> 
#include <stdlib.h> // for utoa
#include "spi.h"

void SPI_config (void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_SPI1);

    // PA4 (CS 1)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_LOW);
    // PA5 (SCK)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetAFPin_0_7 (GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
    // PA6 (CS 2)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_LOW);
    // PA7 (MOSI)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetAFPin_0_7 (GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);

    LL_SPI_SetClockPolarity (SPI1, LL_SPI_POLARITY_HIGH); // Clock to 1 when idle
    LL_SPI_SetClockPhase (SPI1, LL_SPI_PHASE_1EDGE);
    LL_SPI_SetMode (SPI1, LL_SPI_MODE_MASTER);
    LL_SPI_SetBaudRatePrescaler (SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256);
    LL_SPI_SetTransferBitOrder (SPI1, LL_SPI_MSB_FIRST);
    LL_SPI_SetNSSMode (SPI1, LL_SPI_NSS_SOFT);
    LL_SPI_SetTransferDirection (SPI1, LL_SPI_HALF_DUPLEX_TX);
    LL_SPI_SetDataWidth (SPI1, LL_SPI_DATAWIDTH_8BIT);
    LL_SPI_Enable (SPI1);
}

void DAC_initial_setup ()
{
    const uint8_t DAC_setup_sequence [3] = {0b100, 0b0, 0b0}; // set DAC gain to 1

    // DAC 1
    LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_6); // set CS 1 to LOW state

    for (size_t i = 0; i < sizeof (DAC_setup_sequence); i++) 
    {
        LL_SPI_TransmitData8 (SPI1, DAC_setup_sequence [i]);

        /*During discontinuous communications, there is a 2 APB clock period delay between the
        write operation to the SPI_DR register and BSY bit setting. As a consequence it is
        mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
        data.
        */
        while (!LL_SPI_IsActiveFlag_TXE (SPI1));
        while (LL_SPI_IsActiveFlag_BSY (SPI1));
    }

    LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_6); // set CS 1 to HIGH state

    // DAC 2
    LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_4); // set CS 2 to LOW state

    for (size_t i = 0; i < sizeof (DAC_setup_sequence); i++) 
    {
        LL_SPI_TransmitData8 (SPI1, DAC_setup_sequence [i]);

        /*During discontinuous communications, there is a 2 APB clock period delay between the
        write operation to the SPI_DR register and BSY bit setting. As a consequence it is
        mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
        data.
        */
        while (!LL_SPI_IsActiveFlag_TXE (SPI1));
        while (LL_SPI_IsActiveFlag_BSY (SPI1));
    }

    LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_4); // set CS 2 to HIGH state
}

void set_DAC_values (const uint16_t value_1, const uint16_t value_2)
{ 
    const uint8_t update_DAC_DATA = 0b1000; // select DAC DATA register
    
    // DAC 1
    uint8_t DATA_value [2] = {(uint8_t) (value_1 >> 8) & 0xFF, (uint8_t) value_1 & 0xFF}; // {MSByte, LSByte}

    LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_6); // set CS 1 to LOW state

    LL_SPI_TransmitData8 (SPI1, update_DAC_DATA);
    while (!LL_SPI_IsActiveFlag_TXE (SPI1));
    while (LL_SPI_IsActiveFlag_BSY (SPI1));

    for (size_t i = 0; i < sizeof (DATA_value); i++) 
    {
        LL_SPI_TransmitData8 (SPI1, DATA_value [i]);

        /*During discontinuous communications, there is a 2 APB clock period delay between the
        write operation to the SPI_DR register and BSY bit setting. As a consequence it is
        mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
        data.
        */
        while (!LL_SPI_IsActiveFlag_TXE (SPI1));
        while (LL_SPI_IsActiveFlag_BSY (SPI1));
    }

    LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_6); // set CS 1 to HIGH state

    // DAC 2
    DATA_value [0] = (uint8_t) (value_2 >> 8) & 0xFF; // MSByte
    DATA_value [1] = (uint8_t) value_2 & 0xFF; // LSByte

    LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_4); // set CS 2 to LOW state

    LL_SPI_TransmitData8 (SPI1, update_DAC_DATA);
    while (!LL_SPI_IsActiveFlag_TXE (SPI1));
    while (LL_SPI_IsActiveFlag_BSY (SPI1));

    for (size_t i = 0; i < sizeof (DATA_value); i++) 
    {
        LL_SPI_TransmitData8 (SPI1, DATA_value [i]);
        
        /*During discontinuous communications, there is a 2 APB clock period delay between the
        write operation to the SPI_DR register and BSY bit setting. As a consequence it is
        mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
        data.
        */
        while (!LL_SPI_IsActiveFlag_TXE (SPI1));
        while (LL_SPI_IsActiveFlag_BSY (SPI1));
    }

    LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_4); // set CS 2 to HIGH state
}