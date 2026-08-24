/**
 * @file  i2ca_eeprom.h
 * @brief m24m01e_io_t transport bound to F280025 I2CA (polled master mode).
 *
 * Wiring: SDA = GPIO26 (chip pin 43), SCL = GPIO27 (chip pin 44).
 * I2CA is configured as controller @ 400 kHz, FIFO off, by Board_init().
 */

#ifndef I2CA_EEPROM_H
#define I2CA_EEPROM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Return convention matches m24m01e_io_t:
 *   0 = success, M24M01E_ERR_NACK = clean NACK, -1 = bus error / timeout. */
int  i2ca_ep_write(void *ctx, uint8_t addr7,
                   const uint8_t *data, uint16_t len);
int  i2ca_ep_write_read(void *ctx, uint8_t addr7,
                        const uint8_t *wdata, uint16_t wlen,
                        uint8_t *rdata, uint16_t rlen);
void i2ca_ep_delay_ms(void *ctx, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* I2CA_EEPROM_H */
