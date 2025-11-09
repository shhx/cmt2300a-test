#ifndef __CMT_SPI3_H
#define __CMT_SPI3_H

#include <Arduino.h>

#ifdef __cplusplus 
extern "C" { 
#endif

#define SPI_CLOCK_SPEED_HZ 1000000  // 1 MHz
#define CMT2300A_CS_SET(val)    digitalWrite(CMT_CSB_GPIO, val)
#define CMT2300A_FCSB_SET(val)  digitalWrite(CMT_FCSB_GPIO, val)

void cmt_spi_init(void);

void cmt_spi_send(uint8_t data8);
uint8_t cmt_spi_recv(void);

void cmt_spi_write(uint8_t addr, uint8_t dat);
void cmt_spi_read(uint8_t addr, uint8_t* p_dat);

void cmt_spi3_write_fifo(const uint8_t* p_buf, uint16_t len);
void cmt_spi3_read_fifo(uint8_t* p_buf, uint16_t len);

#ifdef __cplusplus 
} 
#endif

#endif
