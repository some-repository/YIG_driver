#include "parse_packet.h"

void parse_packet (uint8_t *buf)
{
    extern uint8_t feedback_request; // defined in main.c
    extern uint8_t DAC_values_modified; // defined in main.c
    extern uint8_t feedback_request; // defined in main.c
    extern uint16_t DAC_1_value, DAC_2_value; // defined in main.c
    extern uint8_t enable_output; // defined in main.c
    extern uint16_t VirtAddVarTab []; // defined in main.c
    extern char utoa_buf []; // defined in main.c
    
    uint8_t header = (uint8_t) buf [0];
    
    if (header & 0b100) // feedback request
    {
        feedback_request = 1;
    }
    else
    {
        DAC_1_value = (uint16_t) buf [2] + (((uint16_t) buf [1]) << 8);
        DAC_2_value = (uint16_t) buf [4] + (((uint16_t) buf [3]) << 8);
        DAC_values_modified = 1;
 
        if (header & 0b10) // write DAC values to EEPROM
        {
            EE_WriteVariable (VirtAddVarTab [0], DAC_1_value);
            EE_WriteVariable (VirtAddVarTab [1], DAC_2_value);
        }
    
        if (header & 0b1)
        {
            enable_output = 1;
        }
        else
        {
            enable_output = 0;
        }
    }
    
    
    print ("header = ");
    print (utoa (header, utoa_buf, 2));
    print (", DAC_1_value = ");
    print (utoa (DAC_1_value, utoa_buf, 10));
    print (", DAC_2_value = ");
    print (utoa (DAC_2_value, utoa_buf, 10));
    print ("\n");
}