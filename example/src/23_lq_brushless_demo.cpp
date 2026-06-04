#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_brushless_demo.cpp
 * @brief   无刷电机输出测试.
 * @author  Copilot
 * @date    2026-05-28
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本例程演示如何通过 ATIM PWM0_PIN81 直接控制无刷电调.
 ********************************************************************************/

/********************************************************************************
 * @brief   无刷电机测试程序.
 * @param   none.
 * @return  none.
 ********************************************************************************/
void lq_brushless_demo(void)
{
    Brushless brushless;

    if (brushless.brushless_init() < 0) {
        lq_log_error("Brushless init failed");
        return;
    }

    while (1) {
        for (int duty = 0; duty <= 100; duty += 10) {
            brushless.set_duty(duty);
            sleep(1);
        }

        for (int duty = 100; duty >= 0; duty -= 10) {
            brushless.set_duty(duty);
            sleep(1);
        }
    }
}