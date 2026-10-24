#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <iostream>
#include <pthread.h>

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#include "shared_memory.hpp"   // ShmData 정의가 들어있는 헤더로 바꾸십시오.

class Joystick
{
public:
    enum class TargetType
    {
        LinVel,
        AngVel,

        AButton,
        BButton,
        XButton,
        YButton,
        LBButton,
        RBButton,
        BackButton,
        StartButton,
        GuideButton,
        L3Button,
        R3Button
    };

    struct ButtonMapping
    {
        uint8_t button_number;
        TargetType target;
        bool invert;
    };
    struct AxisMapping
    {
        uint8_t axis_number;   // joystick axis index
        TargetType target;     // lin_vel_target or ang_vel_target
        uint8_t target_index;  // 0:x, 1:y, 2:z
        float scale;           // gain
        bool invert;           // sign inversion
        float deadband;        // normalized deadband, e.g. 0.05
    };

public:
    Joystick(const char* device_id, ShmData* shared_memory);

    void initialize();
    void start();
    void stop();
    void join();

    float getAxis(uint8_t axis_number) const;
    bool getButton(uint8_t button_number) const;

private:
    static void* thread_function_wrapper(void* context);
    void* loop();

    void processEvent_(const js_event& event);
    void applyAxisMappings_();
    void applyButtonMappings_();

    static float mapAxisValue_(int16_t value);
    static float applyDeadband_(float value, float deadband);

private:
    const char* device_id_;
    ShmData* shared_memory_;

    int js_ = -1;
    pthread_t thread_;
    std::atomic<bool> running_{false};

    static constexpr size_t kMaxAxes = 32;
    static constexpr size_t kMaxButtons = 32;

    std::array<float, kMaxAxes> axes_{};
    std::array<bool, kMaxButtons> buttons_{};


    std::array<AxisMapping, 3> axis_mappings_{{
        {1, TargetType::LinVel, 0, 1.0f, true,  0.05f},
        {0, TargetType::LinVel, 1, 1.0f, true,  0.05f},
        {3, TargetType::AngVel, 2, 1.0f, true,  0.05f},
    }};


    std::array<ButtonMapping, 11> button_mappings_{{
    {0,  TargetType::AButton,     false},  // A
    {1,  TargetType::BButton,     false},  // B
    {2,  TargetType::XButton,     false},  // X
    {3,  TargetType::YButton,     false},  // Y
    {4,  TargetType::LBButton,    false},  // LB
    {5,  TargetType::RBButton,    false},  // RB
    {6,  TargetType::BackButton,  false},  // Back / View
    {7,  TargetType::StartButton, false},  // Start / Menu
    {8,  TargetType::GuideButton, false},  // Xbox / Guide
    {9,  TargetType::L3Button,    false},  // Left stick press
    {10, TargetType::R3Button,    false},  // Right stick press
    }};
};