// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include <stdio.h>

#include "ButtonDriver.hpp"
#include "joyStickDriver.hpp"
#include "MotorDriver.hpp"
#include "ButtonLogic.hpp"
#include "joyStickLogic.hpp"
#include "MotorLogic.hpp"
#include "position.hpp"
#include "control.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_rom_sys.h"

//NOTE: inline constexpr.... creates one of the constant value that is referenced in every file that #includes the file
// with that constexpr. if you use just constexpr.... then every file that #includes that fie with have it's own
//version/copy/thing of that constexpr.

constexpr gpio_num_t enaPinX = GPIO_NUM_18;
constexpr gpio_num_t dirPinX = GPIO_NUM_21;
constexpr gpio_num_t pulsePinX = GPIO_NUM_5;
constexpr gpio_num_t enaPinY = GPIO_NUM_23;
constexpr gpio_num_t dirPinY = GPIO_NUM_22;
constexpr gpio_num_t pulsePinY = GPIO_NUM_4;
constexpr gpio_num_t enaPinZ = GPIO_NUM_25;
constexpr gpio_num_t dirPinZ = GPIO_NUM_26;
constexpr gpio_num_t pulsePinZ = GPIO_NUM_27;
constexpr gpio_num_t jPinX = GPIO_NUM_33;
constexpr gpio_num_t jPinY = GPIO_NUM_35;
constexpr gpio_num_t zPin = GPIO_NUM_32;

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initializing drivers");
    static driver::ButtonDriver butt(zPin);
    static driver::JoyStickDriver stickDrv;
    static driver::MotorDriver shoulderDrv(enaPinX, dirPinX, pulsePinX);  // motor at the origin
    static driver::MotorDriver elbowDrv(enaPinY, dirPinY, pulsePinY);     // motor at the joint between l1 and l2
    static driver::MotorDriver vert(enaPinZ, dirPinZ, pulsePinZ);
    ESP_LOGI(TAG, "Drivers complete");

    ESP_LOGI(TAG, "Initializing logic files");
    static logic::ButtonLogic buttLog(butt);
    static logic::JoyStickLogic stick(stickDrv);
    static logic::MotorLogic shoulder(shoulderDrv);
    static logic::MotorLogic elbow(elbowDrv);
    static logic::MotorLogic vertical(vert);
    static logic::Position pos(26.5f, 26.5f); // l1, l2
    ESP_LOGI(TAG, "Logic files complete");
    ESP_LOGI(TAG, "Initializing arm configuration");
    control::Config cfg;
    ESP_LOGI(TAG, "Arm configuration complete");

    ESP_LOGI(TAG, "Initializing control element");
    static control::Control ctl(buttLog, stick, shoulder, elbow, vertical, pos, cfg);

    ESP_LOGI(TAG, "Debug joystick raw values");
    for (int i = 0; i < 100; i++) {
        printf("raw %d %d  norm %.2f %.2f\n",
               stickDrv.getRawX(), stickDrv.getRawY(), stick.x(), stick.y());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    /*
    cfg.invertShoulder = true;
    cfg.invertElbow = true;
    cfg.elbowAbsolute = false;

    bool joySwapXY = true;
    bool joyInvertX = true;
    bool joyInvertY = false;
*/
    ctl.begin();
    ctl.run();   // never returns

}
