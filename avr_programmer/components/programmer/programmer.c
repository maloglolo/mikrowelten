#include "programmer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#define RESET_PIN   5
#define DMA_CHAN    1
#define TAG         "PROGRAMMER"

static spi_device_handle_t spi;
static uint16_t flash_page_size = 64; // ATtiny85 flash page = 64 bytes

// ----------------------------
// ISP transfer (4-byte)
// ----------------------------
static uint8_t isp_xfer(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
    uint8_t tx_buf[4] = { b1, b2, b3, b4 };
    uint8_t rx_buf[4] = { 0 };

    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    spi_device_transmit(spi, &t);
    return rx_buf[3];  // return last byte
}

// ----------------------------
// Init
// ----------------------------
void programmer_init(void) {
    gpio_reset_pin((gpio_num_t)RESET_PIN);
    gpio_set_direction((gpio_num_t)RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)RESET_PIN, 1);
    esp_rom_delay_us(1000); 

    spi_bus_config_t buscfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    esp_err_t err = spi_bus_initialize(HSPI_HOST, &buscfg, DMA_CHAN);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 50000, // 50 kHz for ATtiny85
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    err = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if (err != ESP_OK) {
        spi = NULL;
        return;
    }
}

// ----------------------------
// Enter/Leave prog mode
// ----------------------------
int programmer_enter(void) {
    gpio_set_level(RESET_PIN, 0);
    esp_rom_delay_us(50000);  // 50 ms low
    gpio_set_level(RESET_PIN, 1);
    esp_rom_delay_us(20000);  // 20 ms high
    uint8_t sig = isp_xfer(0xAC, 0x53, 0x00, 0x00);
    return (sig == 0x53);
}

int programmer_leave(void) {
    gpio_set_level((gpio_num_t)RESET_PIN, 1);
    return 1;
}

// ----------------------------
// Signature & fuse
// ----------------------------
uint8_t programmer_read_signature(uint8_t index) {
    return isp_xfer(0x30, 0x00, index, 0x00);
}

uint8_t programmer_read_fuse(uint8_t addr) {
    return isp_xfer(0x50, 0x00, addr, 0x00);
}

// ----------------------------
// Flash write using page size
// ----------------------------
int programmer_prog_page(uint16_t addr, const uint8_t *data, size_t len) {
    size_t offset = 0;

    while (offset < len) {
        size_t chunk = (len - offset > flash_page_size) ? flash_page_size : (len - offset);

        for (size_t i = 0; i < chunk; i += 2) {
            uint8_t low  = data[offset + i];
            uint8_t high = (i + 1 < chunk) ? data[offset + i + 1] : 0xFF;

            isp_xfer(0x40, (addr >> 8) & 0xFF, addr & 0xFF, low);
            isp_xfer(0x48, (addr >> 8) & 0xFF, addr & 0xFF, high);
            addr++;
        }

        isp_xfer(0x4C, (addr >> 8) & 0xFF, (addr - 1) & 0xFF, 0x00); // commit page
        esp_rom_delay_us(5000); // wait flash write cycle
        offset += chunk;
    }

    return 1;
}

// ----------------------------
// Flash read using page size
// ----------------------------
int programmer_read_page(uint16_t addr, uint8_t *data, size_t len) {
    size_t offset = 0;

    while (offset < len) {
        size_t chunk = (len - offset > flash_page_size) ? flash_page_size : (len - offset);

        for (size_t i = 0; i < chunk; i += 2) {
            data[offset + i]     = isp_xfer(0x20, (addr >> 8) & 0xFF, addr & 0xFF, 0x00);
            data[offset + i + 1] = isp_xfer(0x28, (addr >> 8) & 0xFF, addr & 0xFF, 0x00);
            addr++;
        }

        offset += chunk;
    }

    return 1;
}
