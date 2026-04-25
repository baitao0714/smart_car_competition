#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_motor_encoder_demo.cpp
 * @brief   两个电机 + 两个编码器联测.
 * @author  Claude
 * @date    2026-04-25
 * @version V2.2.0
 * @note    左右电机分别使用 PWM1_PIN65 / PWM2_PIN66 和 PIN_74 / PIN_75.
 *          左右编码器分别使用 ENC_PWM0_PIN64 / PIN_72 和 ENC_PWM3_PIN67 / PIN_73.
 ********************************************************************************/

void lq_motor_encoder_demo(void)
{
    const uint32_t motor_freq_hz = 10000;
    const uint32_t safe_duty = 1500;

    ls_pwm left_motor_pwm(PWM1_PIN65, motor_freq_hz, safe_duty, PWM_POL_INV);
    ls_pwm right_motor_pwm(PWM2_PIN66, motor_freq_hz, safe_duty, PWM_POL_INV);

    ls_gpio left_motor_dir(PIN_74, GPIO_MODE_OUT);
    ls_gpio right_motor_dir(PIN_75, GPIO_MODE_OUT);

    ls_encoder_pwm left_enc(ENC_PWM3_PIN67, PIN_72);
    ls_encoder_pwm right_enc(ENC_PWM0_PIN64, PIN_73);

    left_motor_pwm.pwm_set_duty(safe_duty);
    right_motor_pwm.pwm_set_duty(safe_duty);

    while (ls_system_running.load())
    {
        left_motor_dir.gpio_level_set(GPIO_HIGH);
        right_motor_dir.gpio_level_set(GPIO_HIGH);
        printf("motor+encoder forward, duty=%u/10000\n", safe_duty);

        for (int i = 0; i < 50 && ls_system_running.load(); ++i)
        {
            printf("L:%8.2f R:%8.2f\r", left_enc.encoder_get_count(), right_enc.encoder_get_count());
            fflush(stdout);
            usleep(100000);
        }

        if (!ls_system_running.load())
        {
            break;
        }

        left_motor_dir.gpio_level_set(GPIO_LOW);
        right_motor_dir.gpio_level_set(GPIO_LOW);
        printf("motor+encoder reverse, duty=%u/10000\n", safe_duty);

        for (int i = 0; i < 50 && ls_system_running.load(); ++i)
        {
            printf("L:%8.2f R:%8.2f\r", left_enc.encoder_get_count(), right_enc.encoder_get_count());
            fflush(stdout);
            usleep(100000);
        }
    }

    left_motor_pwm.pwm_set_duty(0);
    right_motor_pwm.pwm_set_duty(0);
    left_motor_dir.gpio_level_set(GPIO_LOW);
    right_motor_dir.gpio_level_set(GPIO_LOW);
    printf("motor+encoder stopped, motors disabled\n");
}