// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include "ButtonLogic.hpp"
#include "joyStickLogic.hpp"
#include "position.hpp"
#include "MotorLogic.hpp"
#include "driver/ledc.h"
#include "iot_servo.h"


namespace control {

struct Config {

    /*
    * start pose (startX, startY): these are in the same units as l1 and l2, and the point has to be reachable.
    * The default (0, 7) only works if |l1 - l2| < 7 < l1 + l2 (roughly). If it’s outside that ring, begin()
    * logs an error and the arm won’t run. The arm also has to be physically placed at that pose at power-up.
    * yMin: it was chosen for a 4/6 arm. Set it to wherever the tip shouldn’t go below, such as the table surface.
    * vMax: it’s in length-units per second, so if the new arm is much bigger or smaller (or you changed units),
    * the same number gives a very different real speed.
    *
    *
    * Reachable positions by the arm: maxY value is 2.65 < x < 52.93
     */


    // tip start position
    float startX = 21.5f;
    float startY = 21.5f;

    float vMax = 8.0f;                  //length units/second
                                        //lower if motor skips under load
    float yMin = 2.0f;                 // keeps tip above this line
    float yMax = 35.0f;
    float xMin = 20.0f;
    float xMax = 40.0f;

    // float yMin = -28.0f;                 // keeps tip above this line
    // float yMax = 28.0f;
    // float xMin = 15.0f;
    // float xMax = 42.0f;

    uint32_t segmentUs = 20000;         // one straight-line segment = 20 ms
    uint32_t maxStepsPerSeg = 64;       // per motor; 16 steps / 20 ms = 800 Hz. must be <= MAX_STEPS
                                        //microstepping 1x @ maxStepsPerSeg = 16 is speed
                                        //microstepping 8x @ maxStepsPerSeg = 16 is speed / 8 (8x slower)
                                        //microstepping 8x @ maxStesPerSeg = 128 is = to speed @ 1x


    // Motor / mechanics. Set according to dip switch
    float stepAngleDeg = 1.8f; //1.8 is confirmed
    int   microstep1 = 16, microstep2 = 16;
    //float gear1 = 40.0f /12.0f, gear2 = 45.0f / 16.0f; //gear1, microstep1 are tied to stepsPerRad1_, one motor shoulder or elbow
    float gear1 = 4.0f, gear2 = 4.0f;
                                        //same for microstep2, gear2
                                        //gear = driven wheel / motor wheel
                                        //diameter of wheel measured at the belt contact point + belt width

    bool invertShoulder = true;        // flip if the joint rotates the wrong way
    bool invertElbow = true;
    bool elbowAbsolute = false;         // true if motor 2 sits at the base and drives link 2 via belt/linkage

    // Joystick orientation
    bool joySwapXY = true;
    bool joyInvertX = true;
    bool joyInvertY = false;

    bool invertZ = true;

    uint16_t maxServoAngle = 180;
    uint16_t maxServoWidth = 2500;
    uint16_t minServoWidth = 500;
    uint32_t servoFrequency = 50;
};

class Control {
    logic::ButtonLogic& button_;
    logic::JoyStickLogic& stick_;
    logic::MotorLogic& shoulder_;
    logic::MotorLogic& elbow_;
    logic::MotorLogic& vert_;
    logic::Position& pos_;
    servo_handle_t& servo_;
    Config cfg_;

    float stepsPerRad1, stepsPerRad2;
    float tipX = 0, tipY = 0;             // where the tip is supposed to be, commanded position
    long steps1 = 0, steps2 = 0;      // absolute step counters (relative to home pose)
    bool ready = false;                //false if home pose was invalid. tick() will do nothing

    //one proposed move
    struct Plan {
        float x, y;
        long s1, s2;
    };

    bool toSteps(float x, float y, long& s1, long& s2) const;
    bool plan(float segmentMoveX, float segmentMoveY, Plan& out) const;

public:
    Control(logic::ButtonLogic& button,
            logic::JoyStickLogic& stick,
            logic::MotorLogic& shoulder,
            logic::MotorLogic& elbow,
            logic::MotorLogic& vert,
            logic::Position& pos,
            servo_handle_t& servo,
            const Config& cfg = Config{});
    void begin();   // enable motors and set the home pose
    void tick();    // runs one segment (blocks ~segmentUs)
    void run();     // tick() forever

    float tipPositionX() const { return tipX; }
    float tipPositionY() const { return tipY; }
};
}
