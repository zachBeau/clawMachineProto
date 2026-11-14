// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include "ButtonLogic.hpp"

#include "esp_log.h"

namespace logic {

    ButtonLogic::ButtonLogic(driver::ButtonDriver& bDriver) : bDriver_(bDriver) {}

    bool ButtonLogic::isPressed() {
        return bDriver_.isPressed();
    }
}
