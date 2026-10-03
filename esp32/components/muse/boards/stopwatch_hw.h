/* Copyright (c) Dehong Hao. SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "driver/i2c_master.h"
#include "muse_board.h"

#ifdef __cplusplus
extern "C" {
#endif

/* M5IOE1 pins are numbered from zero here, unlike M5Stack's PIN_1..PIN_10. */
enum {
    STOPWATCH_MUX = 0,
    STOPWATCH_AUDIO = 2,
    STOPWATCH_TOUCH_RESET = 3,
    STOPWATCH_PANEL_RESET = 4,
    STOPWATCH_PANEL_POWER = 7,
    STOPWATCH_MOTOR = 8,
    STOPWATCH_AMP = 9,
};

esp_err_t stopwatch_hw_init(void);
i2c_master_bus_handle_t stopwatch_i2c_bus(void);
esp_err_t stopwatch_reset_panel(void);
esp_err_t stopwatch_reset_touch(void);
esp_err_t stopwatch_amp_enable(bool enable);
bool stopwatch_amp_enabled(void);
esp_err_t stopwatch_read_power(muse_power_t *out);
esp_err_t stopwatch_power_off(void);
esp_err_t stopwatch_touch_init(void);
bool stopwatch_touch_read(int *x, int *y);
void stopwatch_touch_sleep(void);

#ifdef __cplusplus
}
#endif
