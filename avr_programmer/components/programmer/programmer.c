#include "programmer.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define RESET_PIN 5
#define SPI_HOST    HSPI_HOST
#define DMA_CHAN    1

static spi_device_handle_t spi;

static uint8_t isp_xfer(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
    uint8_t tx_buf[4] = {b1, b2, b3, b4};
    uint8_t rx_buf[4] = {0};

    spi_transaction_t t = {
        .length = 8*4,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };
    spi_device_transmit(spi, &t); // blocking transfer
    return rx_buf[3]; // last byte returned
}

void programmer_init(void) {
    gpio_reset_pin((gpio_num_t)RESET_PIN);
    gpio_set_direction((gpio_num_t)RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)RESET_PIN, 1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4
    };
    spi_bus_initialize(SPI_HOST, &buscfg, DMA_CHAN);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    spi_bus_add_device(SPI_HOST, &devcfg, &spi);
}

int programmer_enter(void) {
    gpio_set_level((gpio_num_t)RESET_PIN, 0);
    esp_rom_delay_us(50);
    uint8_t sig = isp_xfer(0xAC, 0x53, 0x00, 0x00);
    return (sig == 0x53) ? 1 : 0;
}

int programmer_leave(void) {
    gpio_set_level((gpio_num_t)RESET_PIN, 1);
    return 1;
}

uint8_t programmer_read_signature(uint8_t index) {
    return isp_xfer(0x30, 0x00, index, 0x00);
}

int programmer_load_address(uint16_t addr) {
    (void)addr;
    return 1;
}

int programmer_prog_page(uint16_t addr, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i += 2) {
        uint8_t low  = data[i];
        uint8_t high = (i+1 < len) ? data[i+1] : 0xFF;
        isp_xfer(0x40, (addr >> 8) & 0xFF, addr & 0xFF, low);
        isp_xfer(0x48, (addr >> 8) & 0xFF, addr & 0xFF, high);
        addr++;
    }
    isp_xfer(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0x00);
    esp_rom_delay_us(5000);
    return 1;
}

int programmer_read_page(uint16_t addr, uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i += 2) {
        data[i]   = isp_xfer(0x20, (addr >> 8) & 0xFF, addr & 0xFF, 0x00);
        data[i+1] = isp_xfer(0x28, (addr >> 8) & 0xFF, addr & 0xFF, 0x00);
        addr++;
    }
    return 1;
}
