/* Copyright (c) Dehong Hao. SPDX-License-Identifier: Apache-2.0 */
/*
 * Board wiring and initialization follow M5Stack's StopWatch UserDemo V0.5:
 * https://github.com/m5stack/M5StopWatch-UserDemo/tree/V0.5/main/hal
 * Cross-checked against xiaozhi-esp32 at 0d576d3d, main/boards/m5stack/stopwatch.
 * M5Stack's managed M5IOE1/M5PM1 drivers own their register protocols.
 */
#include "stopwatch_hw.h"
#include "cst820.h"
#include "M5IOE1.h"
#include "M5PM1.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "stopwatch";
static constexpr gpio_num_t AMP_GPIO = GPIO_NUM_14;
static constexpr int RAIL_ATTEMPTS = 10;
static constexpr int IO_ATTEMPTS = 3;
static constexpr int BATTERY_EMPTY_MV = 3300;
static constexpr int BATTERY_FULL_MV = 4200;
static i2c_master_bus_handle_t s_bus;
static M5IOE1 s_ioe;
static M5PM1 s_pmic;
static SemaphoreHandle_t s_lock;
static bool s_amp;

static void delay_ms(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* Bound retries for idempotent GPIO and measurement operations. */
template<typename F> static esp_err_t retry(F fn)
{
    for (int i = 0; i < IO_ATTEMPTS; i++) {
        if (fn() == 0) return ESP_OK;
        if (i + 1 < IO_ATTEMPTS) delay_ms(10);
    }
    return ESP_FAIL;
}

static esp_err_t pin_level(int pin, bool high)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = retry([&] {
        m5ioe1_err_t result;
        s_ioe.digitalWriteWithRes(pin, high, &result);
        return result;
    });
    xSemaphoreGive(s_lock);
    return err;
}

static esp_err_t output_pin(int pin)
{
    return retry([&] {
        m5ioe1_err_t result;
        s_ioe.pinModeWithRes(pin, OUTPUT, &result);
        return result;
    });
}

esp_err_t stopwatch_hw_init(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_direction(AMP_GPIO, GPIO_MODE_OUTPUT), TAG, "amp direction");
    ESP_RETURN_ON_ERROR(gpio_set_level(AMP_GPIO, 0), TAG, "amp off");
    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "IOE mutex");
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_io_num = GPIO_NUM_47;
    cfg.scl_io_num = GPIO_NUM_48;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = true;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &s_bus), TAG, "I2C bus");

    /* No interrupt-poll task: none of Muse's inputs use the expander. The
     * driver handles its 0x4F/0x6F address variants and I2C wake handshake. */
    ESP_RETURN_ON_ERROR(retry([] {
        return s_ioe.begin(s_bus, 0x4F, M5IOE1_I2C_FREQ_100K, M5IOE1_INT_MODE_DISABLED);
    }), TAG, "M5IOE1 begin");
    /* Retained GPIO/PWM state can survive an ESP-only reset. Keep the motor
     * off before configuring any rails, including after another firmware. */
    ESP_RETURN_ON_ERROR(pin_level(STOPWATCH_MOTOR, false), TAG, "motor low");
    ESP_RETURN_ON_ERROR(retry([] { return s_ioe.setPwmDuty(0, 0, false, false); }), TAG, "motor PWM off");
    ESP_RETURN_ON_ERROR(retry([] { return s_ioe.setI2cSleepTime(0); }), TAG, "IOE sleep off");
    const int pins[] = { STOPWATCH_MOTOR, STOPWATCH_PANEL_POWER, STOPWATCH_AMP,
        STOPWATCH_TOUCH_RESET, STOPWATCH_PANEL_RESET, STOPWATCH_MUX, STOPWATCH_AUDIO };
    for (int pin : pins) {
        ESP_RETURN_ON_ERROR(output_pin(pin), TAG, "output pin %d", pin);
    }
    const int on[] = { STOPWATCH_PANEL_POWER, STOPWATCH_TOUCH_RESET, STOPWATCH_PANEL_RESET, STOPWATCH_AUDIO };
    for (int pin : on) ESP_RETURN_ON_ERROR(pin_level(pin, true), TAG, "rail %d", pin);
    ESP_RETURN_ON_ERROR(pin_level(STOPWATCH_AMP, false), TAG, "amp off");
    ESP_RETURN_ON_ERROR(pin_level(STOPWATCH_MUX, false), TAG, "mux off");
    /* UserDemo checks L3B after an 80 ms settling interval. Bound its loop
     * so a failed expander cannot hang boot forever. */
    bool rail_ready = false;
    for (int i = 0; i < RAIL_ATTEMPTS; i++) {
        delay_ms(80);
        if (s_ioe.digitalRead(STOPWATCH_PANEL_POWER) == 1) {
            rail_ready = true;
            break;
        }
        ESP_RETURN_ON_ERROR(pin_level(STOPWATCH_PANEL_POWER, true), TAG, "L3B retry");
    }
    ESP_RETURN_ON_FALSE(rail_ready, ESP_ERR_TIMEOUT, TAG, "L3B did not latch");

    ESP_RETURN_ON_ERROR(retry([] { return s_pmic.begin(s_bus); }), TAG, "M5PM1 begin");
    ESP_RETURN_ON_ERROR(retry([] { return s_pmic.setI2cSleepTime(0); }), TAG, "PMIC sleep off");
    ESP_RETURN_ON_ERROR(retry([] { return s_pmic.wdtSet(0); }), TAG, "PMIC watchdog off");
    ESP_RETURN_ON_ERROR(retry([] { return s_pmic.ldoSetPowerHold(true); }), TAG, "RTC rail hold");
    ESP_RETURN_ON_ERROR(retry([] { return s_pmic.setChargeEnable(true); }), TAG, "charge enable");
    ESP_RETURN_ON_ERROR(retry([] {
        return s_pmic.gpioSet(M5PM1_GPIO_NUM_3, M5PM1_GPIO_MODE_OUTPUT, 0,
                             M5PM1_GPIO_PULL_NONE, M5PM1_GPIO_DRIVE_PUSHPULL);
    }), TAG, "charge current");
    ESP_RETURN_ON_ERROR(retry([] {
        return s_pmic.gpioSetFunc(M5PM1_GPIO_NUM_2, M5PM1_GPIO_FUNC_GPIO);
    }), TAG, "CHRG function");
    ESP_RETURN_ON_ERROR(retry([] {
        return s_pmic.gpioSetMode(M5PM1_GPIO_NUM_2, M5PM1_GPIO_MODE_INPUT);
    }), TAG, "CHRG input");
    ESP_RETURN_ON_ERROR(retry([] {
        return s_pmic.gpioSetPull(M5PM1_GPIO_NUM_2, M5PM1_GPIO_PULL_NONE);
    }), TAG, "CHRG pull");
    /* As in UserDemo: a short power-key click must not reset the ESP. The
     * hardware double-click shutdown/download shortcuts are left alone. */
    ESP_RETURN_ON_ERROR(retry([] { return s_pmic.setSingleResetDisable(true); }), TAG, "power key");
    ESP_LOGI(TAG, "M5IOE1/M5PM1 ready; motor off, audio and display powered");
    return ESP_OK;
}

i2c_master_bus_handle_t stopwatch_i2c_bus(void)
{
    return s_bus;
}

static esp_err_t reset_pin(int pin, int low_ms, int high_ms)
{
    ESP_RETURN_ON_ERROR(pin_level(pin, false), TAG, "reset low");
    delay_ms(low_ms);
    ESP_RETURN_ON_ERROR(pin_level(pin, true), TAG, "reset high");
    delay_ms(high_ms);
    return ESP_OK;
}

esp_err_t stopwatch_reset_panel(void)
{
    return reset_pin(STOPWATCH_PANEL_RESET, 20, 120);
}

esp_err_t stopwatch_reset_touch(void)
{
    return reset_pin(STOPWATCH_TOUCH_RESET, 10, 50);
}

esp_err_t stopwatch_amp_enable(bool enable)
{
    esp_err_t err = pin_level(STOPWATCH_AMP, enable);
    /* Fail closed: GPIO14 is a second amp gate, not the I2S data output. */
    gpio_set_level(AMP_GPIO, err == ESP_OK && enable);
    s_amp = err == ESP_OK && enable;
    delay_ms(10);
    return err;
}

bool stopwatch_amp_enabled(void)
{
    return s_amp;
}

esp_err_t stopwatch_read_power(muse_power_t *out)
{
    uint16_t battery, usb;
    uint8_t charge;
    ESP_RETURN_ON_ERROR(retry([&] { return s_pmic.readVbat(&battery); }), TAG, "VBAT");
    ESP_RETURN_ON_ERROR(retry([&] { return s_pmic.readVin(&usb); }), TAG, "VIN");
    ESP_RETURN_ON_ERROR(retry([&] { return s_pmic.gpioGetInput(M5PM1_GPIO_NUM_2, &charge); }), TAG, "CHRG");
    int pct = (int(battery) - BATTERY_EMPTY_MV) * 100 / (BATTERY_FULL_MV - BATTERY_EMPTY_MV);
    out->battery_mv = battery;
    out->battery_pct = battery < 2500 ? -1 : pct < 0 ? 0 : pct > 100 ? 100 : pct;
    out->usb = usb > 4000;
    out->charging = out->usb && !charge;
    return ESP_OK;
}

esp_err_t stopwatch_power_off(void)
{
    ESP_RETURN_ON_ERROR(stopwatch_amp_enable(false), TAG, "amp off");
    /* The vendor method applies the shutdown command's 120 ms delay. Do not
     * retry it: accepting shutdown removes the bus immediately. */
    ESP_RETURN_ON_FALSE(s_pmic.shutdown() == M5PM1_OK, ESP_FAIL, TAG, "shutdown");
    delay_ms(500);
    return ESP_FAIL;
}

static Cst820 s_touch;

esp_err_t stopwatch_touch_init(void)
{
    ESP_RETURN_ON_ERROR(stopwatch_reset_touch(), TAG, "touch reset");
    return s_touch.begin(s_bus) ? ESP_OK : ESP_FAIL;
}

bool stopwatch_touch_read(int *x, int *y)
{
    if (!s_touch.read() || !s_touch.isPressed()) return false;
    *x = s_touch.getX();
    *y = s_touch.getY();
    return true;
}

void stopwatch_touch_sleep(void)
{
    s_touch.sleep();
}
