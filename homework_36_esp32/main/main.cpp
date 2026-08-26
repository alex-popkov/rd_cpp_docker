#include <cstdio>
#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "device_core.hpp"

static constexpr gpio_num_t PIN_SDA = GPIO_NUM_8;
static constexpr gpio_num_t PIN_SCL = GPIO_NUM_9;
static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t REG_PWR1 = 0x6B;
static constexpr uint8_t REG_ACCEL = 0x3B;
static constexpr uart_port_t UART_PORT = UART_NUM_0;

static volatile bool g_tick = false;
static volatile uint32_t g_tick_count = 0;

static void on_timer(void*)
{
    g_tick = true;
    g_tick_count = g_tick_count + 1;
}

static i2c_master_dev_handle_t s_mpu = nullptr;

static esp_err_t mpu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = PIN_SDA;
    bus_cfg.scl_io_num = PIN_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    
    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = MPU_ADDR;
    dev_cfg.scl_speed_hz = 400000;
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &s_mpu));

    // wakeup value = 0
    uint8_t wake[2] = { REG_PWR1, 0x00 };

    return i2c_master_transmit(s_mpu, wake, sizeof(wake), 100);
}

static esp_err_t mpu_read_accel(core::AccelSample& out)
{
    uint8_t reg = REG_ACCEL;
    uint8_t buf[6] = {};
    esp_err_t err = i2c_master_transmit_receive(s_mpu, &reg, 1, buf, sizeof(buf), 100);
    if (err != ESP_OK) return err;

    int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);
    out = core::accel_from_raw(rx, ry, rz);
    return ESP_OK;
}

static void uart_init(void)
{
    uart_config_t cfg = {};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
}

static void handle_command(const char* line, esp_timer_handle_t timer, uint32_t& period_ms)
{
    core::Command cmd = core::parse_command(line);
    switch (cmd.type) {
        case core::CmdType::SetPeriod:
            if (cmd.period_ms >= 20 && cmd.period_ms <= 10000) {
                period_ms = cmd.period_ms;
                esp_timer_stop(timer);
                esp_timer_start_periodic(timer, (uint64_t)period_ms * 1000ULL);
                printf("OK period=%ums\n", (unsigned)period_ms);
            } else {
                printf("ERR period out of range 20..10000\n");
            }
            break;
        default:
            printf("ERR unknown cmd: '%s'\n", line);
            break;
    }
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(mpu_init());
    uart_init();

    const esp_timer_create_args_t timer_args = {
        .callback = &on_timer,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sample_tick",
        .skip_unhandled_events = true
    };
    esp_timer_handle_t timer = nullptr;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));

    uint32_t period_ms = 200;
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, period_ms * 1000ULL));

    char line[64];
    int idx = 0;

    while (1) {
        uint8_t ch;
        while (uart_read_bytes(UART_PORT, &ch, 1, 0) == 1) {
            if (ch == '\n' || ch == '\r') {
                if (idx > 0) {
                    line[idx] = '\0';
                    handle_command(line, timer, period_ms);
                    idx = 0;
                }
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = (char)ch;
            } else {
                idx = 0;
            }
        }

        if (g_tick) {
            g_tick = false;
            int64_t t_ms = esp_timer_get_time() / 1000;

            core::AccelSample a;
            if (mpu_read_accel(a) == ESP_OK) {
                char buf[96];
                core::format_status(buf, sizeof(buf), t_ms, a, period_ms);
                printf("%s\n", buf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}