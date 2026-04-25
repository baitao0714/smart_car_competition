#include "car_runtime.hpp"

#include "lq_common.hpp"

#ifdef LQ_HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

namespace
{
    bool car_runtime_initialized = false;
    bool car_runtime_motor_enabled = false;
    int car_runtime_motor_init_duty = 1000;
}

void CarRuntime_Init(bool enable_motor, int motor_init_duty)
{
    Data_Settings();
    car_runtime_motor_enabled = enable_motor;
    car_runtime_motor_init_duty = motor_init_duty;

    if (enable_motor)
    {
        Motor_Init1(motor_init_duty);
        Motor_Argument();
    }

    car_runtime_initialized = true;
}

void CarRuntime_Shutdown(bool disable_motor)
{
    if (disable_motor && car_runtime_motor_enabled)
    {
        Motor_Disable1();
    }

    cleanup();
    car_runtime_initialized = false;
}

#ifdef LQ_HAVE_OPENCV
bool CarRuntime_ProcessFrame(const cv::Mat &frame, bool enable_motor)
{
    if (!car_runtime_initialized || car_runtime_motor_enabled != enable_motor)
    {
        CarRuntime_Init(enable_motor, car_runtime_motor_init_duty);
    }

    First_image = frame;
    if (First_image.empty())
    {
        return false;
    }

    ImageProcess();
    if (enable_motor)
    {
        Motor_Control();
    }

    return true;
}
#endif

bool CarRuntime_RunCameraLoop(bool enable_motor,
                              uint16_t width,
                              uint16_t height,
                              uint16_t fps,
                              int motor_init_duty,
                              uint32_t empty_frame_delay_us,
                              uint32_t loop_delay_us)
{
#ifndef LQ_HAVE_OPENCV
    (void)enable_motor;
    (void)width;
    (void)height;
    (void)fps;
    (void)motor_init_duty;
    (void)empty_frame_delay_us;
    (void)loop_delay_us;
    lq_log_error("OpenCV not enabled, car runtime is unavailable");
    return false;
#else
    lq_camera cam(width, height, fps);
    if (!cam.is_opened())
    {
        lq_log_error("Failed to open camera for car runtime");
        return false;
    }

    CarRuntime_Init(enable_motor, motor_init_duty);

    while (ls_system_running.load())
    {
        if (!CarRuntime_ProcessFrame(cam.get_raw_frame(), enable_motor))
        {
            usleep(empty_frame_delay_us);
            continue;
        }

        usleep(loop_delay_us);
    }

    CarRuntime_Shutdown(enable_motor);

    return true;
#endif
}
