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
#include "driver/ledc.h"
#include "iot_servo.h"

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

constexpr gpio_num_t servoPin = GPIO_NUM_12;



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
    static logic::Position pos(21.5f, 21.5f, false); // l1, l2
    ESP_LOGI(TAG, "Logic files complete");
    ESP_LOGI(TAG, "Initializing arm configuration");
    control::Config cfg;
    ESP_LOGI(TAG, "Arm configuration complete");

    servo_handle_t servo = NULL;
    servo_config_t servoConfig = SERVO_CONFIG_DEFAULT(
        LEDC_LOW_SPEED_MODE,
        LEDC_TIMER_0,
        LEDC_CHANNEL_0,
        servoPin
    );
    servoConfig.max_angle = cfg.maxServoAngle;
    servoConfig.min_width_us = cfg.minServoWidth;
    servoConfig.max_width_us = cfg.maxServoWidth;
    servoConfig.freq = cfg.servoFrequency;

    ESP_ERROR_CHECK(iot_servo_new(&servoConfig, &servo));


    ESP_LOGI(TAG, "Initializing control element");
    static control::Control ctl(buttLog, stick, shoulder, elbow, vertical, pos, servo, cfg);

    ESP_LOGI(TAG, "Debug joystick raw values");
    for (int i = 0; i < 100; i++) {
        printf("raw %d %d  norm %.2f %.2f\n",
               stickDrv.getRawX(), stickDrv.getRawY(), stick.x(), stick.y());
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ctl.begin();
    ctl.run();   // never returns

}
