#include "input/joystick.hpp"
#include "ipc/shm_utils.hpp"

#include <cerrno>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <yaml-cpp/yaml.h>

/*
 * Xbox Controller index map on Linux joystick API (/dev/input/jsX)
 *
 * 주의:
 * - 아래 인덱스는 일반적인 Xbox 360 / Xbox One / Xbox Series 컨트롤러의
 *   Linux joystick API 기준 매핑입니다.
 * - 드라이버(xpad, xone 등), 연결 방식(USB/Bluetooth), 커널 버전에 따라
 *   axis/button index가 달라질 수 있습니다.
 * - 실제 장치에서는 반드시 다음 명령으로 확인하는 것을 권장합니다.
 *
 *     sudo apt install joystick
 *     jstest /dev/input/js0
 *
 * -------------------------
 * Axis index
 * -------------------------
 * axis 0 : Left stick horizontal
 *          왼쪽 스틱 좌우
 *          보통 left  = -1
 *               right = +1
 *
 * axis 1 : Left stick vertical
 *          왼쪽 스틱 상하
 *          보통 up   = -1
 *               down = +1
 *
 * axis 2 : Left trigger
 *          LT
 *          드라이버에 따라 값 범위가 다를 수 있음
 *          보통 released = -1 또는 0
 *               pressed  = +1
 *
 * axis 3 : Right stick horizontal
 *          오른쪽 스틱 좌우
 *          보통 left  = -1
 *               right = +1
 *
 * axis 4 : Right stick vertical
 *          오른쪽 스틱 상하
 *          보통 up   = -1
 *               down = +1
 *
 * axis 5 : Right trigger
 *          RT
 *          드라이버에 따라 값 범위가 다를 수 있음
 *          보통 released = -1 또는 0
 *               pressed  = +1
 *
 * axis 6 : D-pad horizontal
 *          십자키 좌우
 *          left  = -1
 *          idle  =  0
 *          right = +1
 *
 * axis 7 : D-pad vertical
 *          십자키 상하
 *          up    = -1
 *          idle  =  0
 *          down  = +1
 *
 * -------------------------
 * Button index
 * -------------------------
 * button 0 : A
 * button 1 : B
 * button 2 : X
 * button 3 : Y
 *
 * button 4 : LB
 * button 5 : RB
 *
 * button 6 : Back / View
 * button 7 : Start / Menu
 *
 * button 8 : Xbox / Guide
 *
 * button 9  : Left stick press, L3
 * button 10 : Right stick press, R3
 *
 */

Joystick::Joystick(const char* config_path, ShmData* shared_memory)
    : shared_memory_(shared_memory) {
    const auto config = YAML::LoadFile(config_path);
    device_id_ = config["device"].as<std::string>();
    const auto indices = config["direction_axes"].as<std::vector<int>>();
    const auto invert = config["direction_invert"].as<std::vector<bool>>();
    const double deadband = config["deadband"].as<double>();
    const auto triggers = config["trigger_axes"].as<std::vector<int>>();
    const auto rest = config["trigger_rest"].as<std::vector<double>>();
    const auto pressed = config["trigger_pressed"].as<std::vector<double>>();

    if (device_id_.empty() || indices.size() != 3 || invert.size() != 3 ||
        triggers.size() != 2 || rest.size() != 2 || pressed.size() != 2 ||
        !std::isfinite(deadband) || deadband < 0 || deadband >= 1)
        throw std::runtime_error("invalid joystick mapping configuration");

    for (int i = 0; i < 3; ++i) {
        if (indices[i] < 0 || indices[i] >= int(kMaxAxes))
            throw std::runtime_error("joystick direction axis out of range");

        axis_mappings_[i] = {static_cast<uint8_t>(indices[i]), TargetType::LinVel,
            static_cast<uint8_t>(i), 1.f, invert[i], static_cast<float>(deadband)};
    }

    for (int i = 0; i < 2; ++i) {
        if (triggers[i] < 0 || triggers[i] >= int(kMaxAxes) || !std::isfinite(rest[i]) ||
            !std::isfinite(pressed[i]) || rest[i] < -1 || rest[i] > 1 ||
            pressed[i] < -1 || pressed[i] > 1 || pressed[i] == rest[i])
            throw std::runtime_error("invalid joystick trigger mapping");

        trigger_axes_[i] = triggers[i];
        trigger_rest_[i] = rest[i];
        trigger_pressed_[i] = pressed[i];
    }
}

void Joystick::initialize() {
    running_ = false;

    axes_.fill(0.0f);
    buttons_.fill(false);

    for (int i = 0; i < 2; ++i)
        axes_[trigger_axes_[i]] = trigger_rest_[i];

    js_ = open(device_id_.c_str(), O_RDONLY | O_NONBLOCK);

    if (js_ == -1) {
        std::cerr << "Failed to open joystick device: " << device_id_ << ", errno: " << errno
                  << ", message: " << std::strerror(errno) << std::endl;
        throw std::runtime_error("joystick device open failed");
    }

    std::cout << "Joystick device opened: " << device_id_ << std::endl;
    Publish();
}

void Joystick::start() {
    if (js_ == -1) {
        throw std::runtime_error("joystick device is not open");
    }

    running_ = true;

    const int ret = pthread_create(&thread_, nullptr, Joystick::thread_function_wrapper, this);

    if (ret != 0) {
        thread_ = 0;
        running_ = false;
        std::cerr << "Failed to create joystick thread: " << std::strerror(ret) << std::endl;
        throw std::runtime_error("joystick thread creation failed");
    }
}

void Joystick::stop() {
    running_ = false;
}

void Joystick::join() {
    if (thread_) {
        pthread_join(thread_, nullptr);
        thread_ = 0;
    }

    if (js_ != -1) {
        close(js_);
        js_ = -1;
    }
}

float Joystick::getAxis(uint8_t axis_number) const {
    if (axis_number >= axes_.size()) {
        return 0.0f;
    }

    return axes_[axis_number];
}

bool Joystick::getButton(uint8_t button_number) const {
    if (button_number >= buttons_.size()) {
        return false;
    }

    return buttons_[button_number];
}

float Joystick::mapAxisValue_(int16_t value) {
    /*
     * Linux joystick axis value is approximately [-32767, 32767].
     * int16_t minimum can be -32768, so clamp for safety.
     */
    float normalized = static_cast<float>(value) / 32767.0f;

    if (normalized > 1.0f) {
        normalized = 1.0f;
    } else if (normalized < -1.0f) {
        normalized = -1.0f;
    }

    return normalized;
}

float Joystick::applyDeadband_(float value, float deadband) {
    if (std::fabs(value) < deadband) {
        return 0.0f;
    }

    return value;
}

void* Joystick::thread_function_wrapper(void* context) {
    return static_cast<Joystick*>(context)->loop();
}

void* Joystick::loop() {
    while (running_.load()) {
        js_event event;

        /*
         * O_NONBLOCK에서는 큐가 비어 있으면 read()가 -1을 반환하고
         * errno가 EAGAIN/EWOULDBLOCK가 될 수 있습니다.
         */
        const ssize_t bytes = read(js_, &event, sizeof(event));

        if (bytes == sizeof(event)) {
            processEvent_(event);

            try {
                Publish();
            } catch (const std::exception& error) {
                std::cerr << "[joystick] " << error.what() << std::endl;
                break;
            }
        } else if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            /*
             * 이벤트가 없는 정상 상태입니다.
             * CPU 점유율을 줄이려면 짧게 sleep을 넣는 것이 좋습니다.
             */
            usleep(1000);
        } else if (bytes == -1) {
            std::cerr << "Error reading joystick event: " << std::strerror(errno) << std::endl;
            break;
        } else {
            std::cerr << "Partial joystick event read. bytes = " << bytes << std::endl;
            break;
        }
    }

    running_ = false;

    try {
        Publish(false);
    } catch (const std::exception& error) {
        std::cerr << "[joystick] disconnect publication failed: " << error.what() << std::endl;
    }

    return nullptr;
}

void Joystick::processEvent_(const js_event& event) {
    /*
     * JS_EVENT_INIT bit를 제거해야 axis/button 판별이 깔끔합니다.
     * 예: JS_EVENT_AXIS | JS_EVENT_INIT
     */
    const uint8_t event_type = event.type & ~JS_EVENT_INIT;

    if (event_type == JS_EVENT_AXIS) {
        if (event.number < axes_.size()) {
            axes_[event.number] = mapAxisValue_(event.value);
        }
    } else if (event_type == JS_EVENT_BUTTON) {
        if (event.number < buttons_.size()) {
            buttons_[event.number] = (event.value != 0);
        }
    }
}

void Joystick::Publish(bool connected) {
    ControlData input;

    for (const auto& mapping : axis_mappings_) {
        double value = applyDeadband_(axes_[mapping.axis_number], mapping.deadband);

        if (mapping.invert)
            value = -value;

        const int index = mapping.target == TargetType::AngVel ? 2 : mapping.target_index;
        input.joystick_axes[index] = value * mapping.scale;
    }

    for (const auto& mapping : button_mappings_) {
        const int index = static_cast<int>(mapping.target) - static_cast<int>(TargetType::AButton);
        input.joystick_buttons[index] = buttons_[mapping.button_number] != mapping.invert;
    }

    shm_utils::Lock lock(*shared_memory_);

    if (shared_memory_->stopping || shared_memory_->simulation_finished) {
        running_ = false;
        return;
    }

    auto& next = shm_utils::Next(shared_memory_->control, shared_memory_->control_sequence);
    next.joystick_axes = input.joystick_axes;
    next.joystick_buttons = input.joystick_buttons;
    next.joystick_connected = connected;
    next.joystick_l2 = connected ? std::clamp(
        (axes_[trigger_axes_[0]] - trigger_rest_[0]) /
            (trigger_pressed_[0] - trigger_rest_[0]), 0.0, 1.0) : 0;
    next.joystick_r2 = connected ? std::clamp(
        (axes_[trigger_axes_[1]] - trigger_rest_[1]) /
            (trigger_pressed_[1] - trigger_rest_[1]), 0.0, 1.0) : 0;

    if (!connected) {
        next.joystick_axes.fill(0);
        next.joystick_buttons.fill(false);
    }

    ++next.joystick_sequence;
    next.joystick_published_ns = shm_utils::MonotonicNs();
    ++shared_memory_->control_sequence;
    shm_utils::Notify(*shared_memory_);
}
