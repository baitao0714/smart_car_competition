#ifndef BRUSHLESS_HPP
#define BRUSHLESS_HPP

#include <memory>

#include "lq_drv_inc.hpp"

class Brushless
{
public:
    Brushless();
    ~Brushless();

    int brushless_init(void);
    void set_duty(int value);
    int get_duty(void);

private:
    uint32_t duty_to_pwm_duty(int value) const;

private:
    std::unique_ptr<ls_pwm> pwm;
    int duty;
    bool initialized;
};

#endif