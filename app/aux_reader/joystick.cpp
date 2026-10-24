#include "joystick.hpp"

#include <cerrno>
#include <cstring>
#include <cmath>

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




Joystick::Joystick(const char* device_id, ShmData* shared_memory)
:
    device_id_(device_id),
    shared_memory_(shared_memory)
{
}

void Joystick::initialize()
{
    running_ = false;

    axes_.fill(0.0f);
    buttons_.fill(false);

    js_ = open(device_id_, O_RDONLY | O_NONBLOCK);

    if (js_ == -1)
    {
        std::cerr << "Failed to open joystick device: "
                  << device_id_
                  << ", errno: "
                  << errno
                  << ", message: "
                  << std::strerror(errno)
                  << std::endl;
        return;
    }

    std::cout << "Joystick device opened: " << device_id_ << std::endl;
}

void Joystick::start()
{
    if (js_ == -1)
    {
        std::cerr << "Joystick device is not opened. start() ignored." << std::endl;
        return;
    }

    running_ = true;

    const int ret = pthread_create(&thread_, nullptr, Joystick::thread_function_wrapper, this);
    if (ret != 0)
    {
        running_ = false;
        std::cerr << "Failed to create joystick thread: "
                  << std::strerror(ret)
                  << std::endl;
    }
}

void Joystick::stop()
{
    running_ = false;
}

void Joystick::join()
{
    if (thread_)
    {
        pthread_join(thread_, nullptr);
    }

    if (js_ != -1)
    {
        close(js_);
        js_ = -1;
    }
}

float Joystick::getAxis(uint8_t axis_number) const
{
    if (axis_number >= axes_.size())
    {
        return 0.0f;
    }

    return axes_[axis_number];
}

bool Joystick::getButton(uint8_t button_number) const
{
    if (button_number >= buttons_.size())
    {
        return false;
    }

    return buttons_[button_number];
}

float Joystick::mapAxisValue_(int16_t value)
{
    /*
     * Linux joystick axis value is approximately [-32767, 32767].
     * int16_t minimum can be -32768, so clamp for safety.
     */
    float normalized = static_cast<float>(value) / 32767.0f;

    if (normalized > 1.0f)
    {
        normalized = 1.0f;
    }
    else if (normalized < -1.0f)
    {
        normalized = -1.0f;
    }

    return normalized;
}

float Joystick::applyDeadband_(float value, float deadband)
{
    if (std::fabs(value) < deadband)
    {
        return 0.0f;
    }

    return value;
}

void* Joystick::thread_function_wrapper(void* context)
{
    return static_cast<Joystick*>(context)->loop();
}

void* Joystick::loop()
{
    while (running_.load())
    {
        js_event event;

        /*
         * O_NONBLOCK에서는 큐가 비어 있으면 read()가 -1을 반환하고
         * errno가 EAGAIN/EWOULDBLOCK가 될 수 있습니다.
         */
        const ssize_t bytes = read(js_, &event, sizeof(event));

        if (bytes == sizeof(event))
        {
            processEvent_(event);
            applyAxisMappings_();
            applyButtonMappings_();
        }
        else if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            /*
             * 이벤트가 없는 정상 상태입니다.
             * CPU 점유율을 줄이려면 짧게 sleep을 넣는 것이 좋습니다.
             */
            usleep(1000);
        }
        else if (bytes == -1)
        {
            std::cerr << "Error reading joystick event: "
                      << std::strerror(errno)
                      << std::endl;
            break;
        }
        else
        {
            std::cerr << "Partial joystick event read. bytes = "
                      << bytes
                      << std::endl;
            break;
        }
    }

    return nullptr;
}

void Joystick::processEvent_(const js_event& event)
{
    /*
     * JS_EVENT_INIT bit를 제거해야 axis/button 판별이 깔끔합니다.
     * 예: JS_EVENT_AXIS | JS_EVENT_INIT
     */
    const uint8_t event_type = event.type & ~JS_EVENT_INIT;

    if (event_type == JS_EVENT_AXIS)
    {
        if (event.number < axes_.size())
        {
            axes_[event.number] = mapAxisValue_(event.value);
        }
    }
    else if (event_type == JS_EVENT_BUTTON)
    {
        if (event.number < buttons_.size())
        {
            buttons_[event.number] = (event.value != 0);
        }
    }
}

void Joystick::applyAxisMappings_()
{
    if (shared_memory_ == nullptr)
    {
        return;
    }

    for (const AxisMapping& mapping : axis_mappings_)
    {
        if (mapping.axis_number >= axes_.size())
        {
            continue;
        }

        float value = axes_[mapping.axis_number];

        value = applyDeadband_(value, mapping.deadband);

        if (mapping.invert)
        {
            value = -value;
        }

        value *= mapping.scale;

        switch (mapping.target)
        {
        case TargetType::LinVel:
            if (mapping.target_index < 3)
            {
                shared_memory_->lin_vel_target[mapping.target_index] = value;
            }
            break;

        case TargetType::AngVel:
            if (mapping.target_index < 3)
            {
                shared_memory_->ang_vel_target[mapping.target_index] = value;
            }
            break;
        }
    }
}

void Joystick::applyButtonMappings_()
{
    if (shared_memory_ == nullptr)
    {
        return;
    }

    for (const ButtonMapping& mapping : button_mappings_)
    {
        if (mapping.button_number >= buttons_.size())
        {
            continue;
        }

        bool value = buttons_[mapping.button_number];

        if (mapping.invert)
        {
            value = !value;
        }

        switch (mapping.target)
        {
        case TargetType::AButton:
            shared_memory_->a_button = value;
            break;

        case TargetType::BButton:
            shared_memory_->b_button = value;
            break;

        case TargetType::XButton:
            shared_memory_->x_button = value;
            break;

        case TargetType::YButton:
            shared_memory_->y_button = value;
            break;

        case TargetType::LBButton:
            shared_memory_->lb_button = value;
            break;

        case TargetType::RBButton:
            shared_memory_->rb_button = value;
            break;

        case TargetType::BackButton:
            shared_memory_->back_button = value;
            break;

        case TargetType::StartButton:
            shared_memory_->start_button = value;
            break;

        case TargetType::GuideButton:
            shared_memory_->guide_button = value;
            break;

        case TargetType::L3Button:
            shared_memory_->l3_button = value;
            break;

        case TargetType::R3Button:
            shared_memory_->r3_button = value;
            break;

        default:
            break;
        }
    }
}