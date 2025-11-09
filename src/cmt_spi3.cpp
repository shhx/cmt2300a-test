#include "cmt_spi3.h"

#include <Arduino.h>
#include <driver/spi_master.h>
#include <string.h>
#include "esp_heap_caps.h"

#include "driver/gpio.h"
#include "gpio_defs.h"

static spi_device_handle_t g_spi_handle = NULL;

void cmt_spi_init(void) {
    pinMode(CMT_CSB_GPIO, OUTPUT);
    pinMode(CMT_FCSB_GPIO, OUTPUT);
    CMT2300A_CS_SET(1);
    CMT2300A_FCSB_SET(1);

    spi_bus_config_t bus_config = {
        .mosi_io_num = CMT_SDIO_GPIO,  // MOSI pin (used for both TX and RX in 3-wire mode)
        .miso_io_num = -1,  // Not used in 3-wire mode
        .sclk_io_num = CMT_SCLK_GPIO,      // Clock pin
        .quadwp_io_num = -1,               // Not used
        .quadhd_io_num = -1,               // Not used
        .max_transfer_sz = 4096,           // Maximum transfer size in bytes
        .flags = SPICOMMON_BUSFLAG_MASTER  // Master mode
    };

    // Initialize SPI bus on SPI2 (HSPI)
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("Failed to initialize SPI bus: %s\n",
                      esp_err_to_name(ret));
        return;
    }
    Serial.println("SPI bus initialized successfully");

    // Configure SPI device with 3-wire and half-duplex flags
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,      // Optional: command phase bits
        .address_bits = 0,      // Optional: address phase bits
        .dummy_bits = 0,        // Optional: dummy bits
        .mode = 0,              // SPI mode 0 (CPOL=0, CPHA=0)
        .duty_cycle_pos = 0,    // 50% duty cycle
        .cs_ena_pretrans = 0,   // CS setup time
        .cs_ena_posttrans = 0,  // CS hold time
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
        .input_delay_ns = 0,             // Input delay
        .spics_io_num = -1,              // CS pin
        .flags = SPI_DEVICE_3WIRE |      // 3-wire mode: MOSI for both TX and RX
                 SPI_DEVICE_HALFDUPLEX,  // Half-duplex mode: TX then RX
        .queue_size = 7,                 // Transaction queue size
        .pre_cb = NULL,                  // Pre-transfer callback
        .post_cb = NULL                  // Post-transfer callback
    };

    // Add device to SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &dev_config, &g_spi_handle);
    if (ret != ESP_OK) {
        Serial.printf("Failed to add device to SPI bus: %s\n",
                      esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return;
    }
    Serial.println("SPI device added successfully with 3-wire half-duplex mode");
}

void cmt_spi_send(uint8_t data8) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 8; /* bits */
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = data8;

    CMT2300A_CS_SET(0);
    esp_err_t ret = spi_device_transmit(g_spi_handle, &t);
    CMT2300A_CS_SET(1);
    if (ret != ESP_OK) {
        Serial.printf("cmt_spi_send: spi_device_transmit failed: %s\n", esp_err_to_name(ret));
    }
}

uint8_t cmt_spi_recv(void) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    /* For half-duplex RX-only transactions use rxlength (bits) */
    t.length = 0;
    t.rxlength = 8; /* bits */
    t.flags = SPI_TRANS_USE_RXDATA;

    CMT2300A_CS_SET(0);
    esp_err_t ret = spi_device_transmit(g_spi_handle, &t);
    CMT2300A_CS_SET(1);
    if (ret != ESP_OK) {
        Serial.printf("cmt_spi_recv: spi_device_transmit failed: %s\n", esp_err_to_name(ret));
        return 0xFF;
    }

    return t.rx_data[0];
}

void cmt_spi_write(uint8_t addr, uint8_t dat) {
    /* Write register: send address byte then data byte in one TX transaction */
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 16; /* 2 bytes = 16 bits */
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = addr;
    t.tx_data[1] = dat;

    CMT2300A_CS_SET(0);
    esp_err_t ret = spi_device_transmit(g_spi_handle, &t);
    CMT2300A_CS_SET(1);
    if (ret != ESP_OK) {
        Serial.printf("cmt_spi_write: spi_device_transmit failed: %s\n", esp_err_to_name(ret));
    }
}

void cmt_spi_read(uint8_t addr, uint8_t* p_dat) {
    if (p_dat == NULL) return;

    /* Send address (TX phase) */
    spi_transaction_t t_tx;
    memset(&t_tx, 0, sizeof(t_tx));
    t_tx.length = 8;
    t_tx.flags = SPI_TRANS_USE_TXDATA;
    t_tx.tx_data[0] = addr | 0x80;  // Set MSB for read operation

    CMT2300A_CS_SET(0);
    esp_err_t ret = spi_device_transmit(g_spi_handle, &t_tx);
    if (ret != ESP_OK) {
        Serial.printf("cmt_spi_read: addr tx failed: %s\n", esp_err_to_name(ret));
        *p_dat = 0xFF;
        return;
    }

    /* Receive data (RX phase) */
    spi_transaction_t t_rx;
    memset(&t_rx, 0, sizeof(t_rx));
    /* RX-only in half-duplex: set rxlength */
    t_rx.length = 8;
    t_rx.rxlength = 8;
    t_rx.flags = SPI_TRANS_USE_RXDATA;

    ret = spi_device_transmit(g_spi_handle, &t_rx);
    CMT2300A_CS_SET(1);
    if (ret != ESP_OK) {
        Serial.printf("cmt_spi_read: data rx failed: %s\n", esp_err_to_name(ret));
        *p_dat = 0xFF;
        return;
    }

    *p_dat = t_rx.rx_data[0];
}

void cmt_spi3_write_fifo(const uint8_t dat) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8; // Length in bits
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = dat;
    CMT2300A_FCSB_SET(0);
    esp_err_t ret = spi_device_transmit(g_spi_handle, &t);
    CMT2300A_FCSB_SET(1);
    if (ret != ESP_OK) {
        Serial.printf("cmt_spi3_write_fifo: spi_device_transmit failed: %s\n", esp_err_to_name(ret));
    }
}

void cmt_spi3_write_fifo(const uint8_t* p_buf, uint16_t len) {
    if (p_buf == NULL || len == 0) return;
    for (uint16_t i = 0; i < len; ++i) {
        cmt_spi3_write_fifo(p_buf[i]);
    }
}

void cmt_spi3_read_fifo(uint8_t* p_buf, uint16_t len) {
    if (p_buf == NULL || len == 0) return;
    CMT2300A_FCSB_SET(0);

    /* Allocate DMA-capable buffer for RX because spi driver may use DMA */
    uint8_t* rx_dma = (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_DMA);
    if (rx_dma == NULL) {
        /* Fallback to byte-by-byte reads */
        for (uint16_t i = 0; i < len; ++i) {
            p_buf[i] = cmt_spi_recv();
        }
        CMT2300A_FCSB_SET(1);
        return;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 0;          // no TX bits
    t.rxlength = len * 8;  // RX bits (half-duplex)
    t.rx_buffer = rx_dma;

    esp_err_t ret = spi_device_transmit(g_spi_handle, &t);
    CMT2300A_FCSB_SET(1);
    if (ret != ESP_OK) {
        Serial.printf("cmt_spi3_read_fifo: spi_device_transmit failed: %s\n", esp_err_to_name(ret));
        /* fallback */
        for (uint16_t i = 0; i < len; ++i) {
            p_buf[i] = cmt_spi_recv();
        }
        heap_caps_free(rx_dma);
        return;
    }

    memcpy(p_buf, rx_dma, len);
    heap_caps_free(rx_dma);
}
