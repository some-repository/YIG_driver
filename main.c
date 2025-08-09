#define MCO

#include "stm32f411xe.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_flash.h"
#include <stddef.h> 
#include <stdlib.h> // for utoa
#include "print.h"
#include "usb.h"
#include "spi.h"
#include "eeprom.h"

char utoa_buf [16] = {0};
const uint8_t initial_delay = 15;
uint8_t enable_output = 0; // logic variable, responsible for analog part power
uint8_t ready = 0; // logic variable which is setted with delay after turn on
uint8_t DAC_values_modified = 0; // logic variable
uint8_t feedback_request = 0; // logical variable
uint16_t DAC_1_value = 0, DAC_2_value = 0;
const uint16_t VirtAddVarTab [NB_OF_VAR] = {1, 2};

void RCC_config (void);
void MCO_config (void);
void ADC_config (void);
void GPIO_config (void);
void SysTick_config (void);
void parse_packet (uint8_t *buf);

void GPIO_config (void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB);
    
    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT); // analog part power on/off
    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT); // auxiliary LED
    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT); // output 2 fail LED
    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT); // output 2 ok LED
    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT); // output 1 fail LED
    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT); // output 1 ok LED
    
    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT); // enable button input
    LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_NO); // enable button input has external pull-up

    LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT); // external power check input
    LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP); // LTC4418 has open-drain outputs

    // configure ADC inputs
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    // PA1
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
    // PA2
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
}

void MCO_config (void) // SYSCLK/2 on PA8
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_0);
    LL_RCC_ConfigMCO (LL_RCC_MCO1SOURCE_PLLCLK, LL_RCC_MCO1_DIV_4); //only MCO1 is available on this MCU
}

void RCC_config (void) // CPU clock frequency is 96 MHz
{
    LL_RCC_DeInit ();
    LL_FLASH_SetLatency (LL_FLASH_LATENCY_2);
    #if defined (MCO)
        MCO_config ();
    #endif //MCO

    /* Enable HSE and wait for activation*/
    LL_RCC_HSE_Enable ();
    while (LL_RCC_HSE_IsReady () != 1);

    LL_RCC_PLL_Disable ();
    LL_RCC_PLL_ConfigDomain_SYS (LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 384, LL_RCC_PLLP_DIV_4); //384 is PLLN
    LL_RCC_PLL_ConfigDomain_48M (LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 384, LL_RCC_PLLQ_DIV_8);
    LL_RCC_PLL_Enable ();
    while (LL_RCC_PLL_IsReady () != 1);

    LL_RCC_SetAHBPrescaler (LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler (LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler (LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource (LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource () != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
    while (LL_RCC_GetUSBClockFreq (LL_RCC_USB_CLKSOURCE) == LL_RCC_PERIPH_FREQUENCY_NO);
}

void ADC_config (void)
{
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_ADC1);
    LL_ADC_SetCommonClock (__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV8);

    // ADC setup
    LL_ADC_Enable (ADC1);
    LL_ADC_SetResolution (ADC1, LL_ADC_RESOLUTION_12B);  // 12 bit resolution
    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT); // fill by zeros on the left
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_480CYCLES); // select the longest
    LL_ADC_INJ_SetTriggerSource (ADC1, LL_ADC_INJ_TRIG_SOFTWARE); // software start of conversion
    LL_ADC_INJ_SetTrigAuto (ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);
    LL_ADC_SetSequencersScanMode (ADC1, LL_ADC_SEQ_SCAN_ENABLE);
    LL_ADC_INJ_SetSequencerLength (ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS); // two injected channels are in use
    LL_ADC_INJ_SetSequencerRanks (ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1); // channel 1 (PA1)
    LL_ADC_INJ_SetSequencerRanks (ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_2); // channel 2 (PA2)
}

void SysTick_config (void)
{
    const uint32_t ticks_per_second = 100;
    LL_InitTick (96000000 / 8, ticks_per_second);
    LL_SYSTICK_SetClkSource (LL_SYSTICK_CLKSOURCE_HCLK_DIV8);
    // Interrupt
    NVIC_SetPriority (SysTick_IRQn, 1); // higher priority than OTG_FS_IRQn
    LL_SYSTICK_EnableIT ();
}

void OTG_FS_IRQHandler (void)
{
    USB_interrupt_handler ();
}

void SysTick_Handler (void)
{
    static uint8_t debouncing_counter = 0; // persisting local variable for enable button sofware debouncing
    static uint8_t initial_counter = 0;
    
    if (initial_counter < initial_delay) // startup period
    {
        initial_counter++;
        if (initial_counter == initial_delay)
        {
            DAC_initial_setup ();
            print ("DAC_initial_setup() done\n");
            set_DAC_values (0, 0);
            print ("set_DAC_values(0, 0) done\n");

            LL_GPIO_SetOutputPin (GPIOB, LL_GPIO_PIN_6); // turn on LED
            ready = 1;
            LL_ADC_INJ_StartConversionSWStart (ADC1); // start AD conversion
        }

        /*print ("SysTick interrupt, initial_counter = ");
        print (utoa (initial_counter, utoa_buf, 10));
        print ("\n");*/
    }
    else // startup period ended
    {      
        uint32_t port_read = LL_GPIO_ReadInputPort (GPIOB);
        uint32_t PB7 = port_read & (1 << 7); // external power check input
        uint32_t PB8 = port_read & (1 << 8); // enable button input

        if (PB8 == 0) // enable button pressed
        {
            debouncing_counter = 10; // load initial value
        }
        if ((PB8 != 0) && (debouncing_counter > 0)) // enable button released recently
        {
            debouncing_counter--;

            if (debouncing_counter == 0)
            {
                // toggle enable_output variable
                if (enable_output == 1)
                {
                    enable_output = 0;
                }
                else
                {
                    enable_output = 1;
                }
            }
        }

        if (DAC_values_modified == 1)
        {
            set_DAC_values (DAC_1_value, DAC_2_value);
            DAC_values_modified = 0;
            print ("DAC values updated\n");
        }

        if (enable_output == 1)
        {
            LL_GPIO_SetOutputPin (GPIOB, LL_GPIO_PIN_9); // turn on analog part power
        }
        else
        {
            LL_GPIO_ResetOutputPin (GPIOB, LL_GPIO_PIN_9); // turn off analog part power
        }

        while (LL_ADC_IsActiveFlag_JEOS (ADC1) == 0);
        uint16_t ADC_value_1 = LL_ADC_INJ_ReadConversionData12 (ADC1, LL_ADC_INJ_RANK_1);
        uint16_t ADC_value_2 = LL_ADC_INJ_ReadConversionData12 (ADC1, LL_ADC_INJ_RANK_2);
        LL_ADC_INJ_StartConversionSWStart (ADC1); // start new AD conversion

        /*print ("ADC_value_1 =");
        print (utoa (ADC_value_1, utoa_buf, 10));
        print (", ADC_value_2 =");
        print (utoa (ADC_value_2, utoa_buf, 10));
        print ("\n");*/

        if (feedback_request == 1)
        {
            *((uint16_t *) &bufTX [0]) = ADC_value_1;
            *((uint16_t *) &bufTX [2]) = ADC_value_2;
            send_ep (1, bufTX, 4); // Send 4 bytes of data to EP1
            
            /*print ("bufTX [0] =");
            print (utoa (*((uint16_t *) &bufTX [0]), utoa_buf, 10));
            print (", bufTX [2] =");
            print (utoa (*((uint16_t *) &bufTX [2]), utoa_buf, 10));
            print ("\n");*/
            feedback_request = 0;
        }
    }
}

int main (void)
{
    GPIO_config ();
    LL_GPIO_ResetOutputPin (GPIOB, LL_GPIO_PIN_9); // turn off analog part power
    RCC_config ();
    UART_config (96000000, 115200); // peripheral clock frequency, baudrate
    print ("Print test\n");
    
    FLASH_Unlock (); // Unlock the Flash Program Erase controller
    EE_Init (); // EEPROM Init
    // check existence and read DAC values from EEPROM to corresponding variables
    if (EE_ReadVariable (VirtAddVarTab [0], &DAC_1_value) == 1)
    {
        DAC_1_value = 0; // set DAC value to 0 if EEPROM variable doesn't exist
    } 
    if (EE_ReadVariable (VirtAddVarTab [1], &DAC_2_value) == 1)
    {
        DAC_2_value = 0; // set DAC value to 0 if EEPROM variable doesn't exist
    }
    DAC_values_modified = 1;

    SPI_config ();
    ADC_config ();
    USB_config ();
    SysTick_config ();
    
    print ("Initial DAC_1_value = ");
    print (utoa (DAC_1_value, utoa_buf, 10));
    print (", initial DAC_2_value = ");
    print (utoa (DAC_2_value, utoa_buf, 10));
    print ("\n");
    
    while (1) 
    {
       
    }
}