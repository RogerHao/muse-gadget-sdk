/* Copyright (c) Dehong Hao. SPDX-License-Identifier: Apache-2.0 */
#include "cst820.h"
#include <cassert>
#include <cstring>

static uint8_t report[7];
static bool fail_read, zero_id;
static int created, removed, slept;

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t *cfg,
                                   i2c_master_dev_handle_t *out)
{
    assert(cfg->device_address == 0x15);
    assert(cfg->scl_speed_hz == 100000);
    *out = reinterpret_cast<void *>(1);
    created++;
    return ESP_OK;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t)
{
    removed++;
    return ESP_OK;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t *reg, size_t,
                                    uint8_t *out, size_t len, int timeout)
{
    assert(timeout > 0);  // A disconnected controller cannot block LVGL indefinitely.
    if (fail_read) return ESP_FAIL;
    if (*reg == 0) {
        assert(len == sizeof(report));
        memcpy(out, report, len);
    } else {
        assert(*reg == 0xA7 || *reg == 0xA9);
        *out = zero_id ? 0 : 1;
    }
    return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t *data, size_t len, int timeout)
{
    assert(timeout > 0);
    assert(len == 2 && data[0] == 0xE5 && data[1] == 3);
    slept++;
    return ESP_OK;
}

int main()
{
    {
        Cst820 touch;
        assert(touch.begin(nullptr));
        // Touch-down at (300, 465), with coordinate high bits and event bits.
        report[2] = 1; report[3] = 1; report[4] = 44; report[5] = 1; report[6] = 209;
        assert(touch.read() && touch.isPressed());
        assert(touch.getX() == 300 && touch.getY() == 465);
        report[3] = 0x81;  // Contact/move must stay pressed.
        assert(touch.read() && touch.isPressed() && touch.getX() == 300);
        report[3] = 0x41;  // Up may still report a finger, but is released.
        assert(touch.read() && !touch.isPressed());
        report[3] = 1; report[2] = 0;
        assert(touch.read() && !touch.isPressed());
        fail_read = true;
        assert(!touch.read());
        fail_read = false;
        touch.sleep();
        assert(slept == 1);
    }
    assert(created == removed);
    {
        zero_id = true;
        Cst820 absent;
        assert(!absent.begin(nullptr));
        assert(created == removed);  // Failed probe immediately frees the handle.
    }
    assert(created == removed);      // No double removal in the destructor.
}
