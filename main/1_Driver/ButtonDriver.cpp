// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include "ButtonDriver.hpp"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static const char* TAG = "BUTTON_DRIVER";

namespace driver {
    ButtonDriver::ButtonDriver(gpio_num_t buttonPin) : buttonPin(buttonPin) {}

    void ButtonDriver::init() {
        ESP_LOGI(TAG, "Button Initialization");
        //Configure GPIO pins
        gpio_config_t bcfg = {};
        bcfg.pin_bit_mask = (1ULL << buttonPin);
        bcfg.mode = GPIO_MODE_INPUT;
        bcfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        bcfg.pull_up_en = GPIO_PULLUP_DISABLE;
        bcfg.intr_type = GPIO_INTR_DISABLE;

        gpio_config(&bcfg);

    }

    bool ButtonDriver::isPressed() {
        ESP_LOGI(TAG, "called isPressed()");
        return gpio_get_level(buttonPin) == 1;
    }
}