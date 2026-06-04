#include "brushless.hpp"

#include <algorithm>
#include <memory>

namespace {
constexpr pwm_pin_t kBrushlessPin = PWM2_PIN88;
constexpr uint32_t kBrushlessPwmFreqHz = 50;
constexpr uint32_t kBrushlessMinPulseUs = 1000;
constexpr uint32_t kBrushlessMaxPulseUs = 2000;

int ClampPercent(int value) {
    if (value < 0) {
        return 0;
    }
    if (value > 100) {
        return 100;
    }
    return value;
}
} // namespace

Brushless::Brushless()
    : duty(0), initialized(false)
{
}

Brushless::~Brushless()
{
    pwm.reset();
}

int Brushless::brushless_init(void)
{
    if (initialized && pwm) {
        return 0;
    }

    pwm = std::make_unique<ls_pwm>(kBrushlessPin, kBrushlessPwmFreqHz, 0);
    initialized = static_cast<bool>(pwm);
    if (!initialized) {
        return -1;
    }

    set_duty(0);
    return 0;
}

uint32_t Brushless::duty_to_pwm_duty(int value) const
{
    const int clamped = ClampPercent(value);
    const uint32_t pulse_us = kBrushlessMinPulseUs +
                               static_cast<uint32_t>(clamped) *
                                   (kBrushlessMaxPulseUs - kBrushlessMinPulseUs) /
                                   100U;
    const uint32_t period_us = 1000000U / kBrushlessPwmFreqHz;
    return pulse_us * PWM_DUTY_MAX / period_us;
}

void Brushless::set_duty(int value)
{
    if (!initialized || !pwm) {
        std::cout << "设备未初始化!!!" << std::endl;
        return;
    }

    value = ClampPercent(value);
    pwm->pwm_set_duty(duty_to_pwm_duty(value));
    duty = value;
}

int Brushless::get_duty(void)
{
    return duty;
}