/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPI_H
#define SPI_H
/* Define to prevent recursive inclusion -------------------------------------*/

void SPI_config (void);
void DAC_initial_setup (void);
void set_DAC_values (const uint16_t value_1, const uint16_t value_2);

#endif /* SPI_H */