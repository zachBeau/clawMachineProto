// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "driver/gpio.h"
#include "soc/gpio_num.h"

namespace driver {

class ButtonDriver {
    gpio_num_t buttonPin;
public:
    ButtonDriver(gpio_num_t);
    void init();
    bool isPressed();
};
}