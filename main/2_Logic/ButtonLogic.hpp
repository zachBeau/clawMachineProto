// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include "ButtonDriver.hpp"

namespace logic {

class ButtonLogic {
    driver::ButtonDriver& bDriver_;
    bool buttonPressed;
public:
    ButtonLogic(driver::ButtonDriver& bDriver);
    bool isPressed();
};
}