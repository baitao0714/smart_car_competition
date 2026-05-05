#include "car_runtime.hpp"

#include "lq_common.hpp"

#include <chrono>
#include <algorithm>
#ifdef LQ_HAVE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

#ifdef LQ_HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

namespace
{
    #ifdef LQ_HAVE_OPENCV
        static bool CarRuntime_BuildGrayOverlayFrame(cv::Mat &out_bgr)
        {
            if (Gray_image.empty())
            {
                return false;
            }

            // 灰度图转三通道，便于彩色画线
            cv::cvtColor(Gray_image, out_bgr, cv::COLOR_GRAY2BGR);

            const int rows = out_bgr.rows;
            const int cols = out_bgr.cols;
            const int off_line = std::max(0, std::min((int)ImageStatus.OFFLine, rows - 1));

            for (int y = rows - 1; y > off_line; --y)
            {
                int l = std::max(0, std::min((int)ImageDeal[y].LeftBorder, cols - 1));
                int r = std::max(0, std::min((int)ImageDeal[y].RightBorder, cols - 1));
                int c = std::max(0, std::min((int)ImageDeal[y].Center, cols - 1));

                // 左边线: 绿
                out_bgr.at<cv::Vec3b>(y, l) = cv::Vec3b(0, 255, 0);
                // 右边线: 红
                out_bgr.at<cv::Vec3b>(y, r) = cv::Vec3b(0, 0, 255);
                // 中线: 黄
                out_bgr.at<cv::Vec3b>(y, c) = cv::Vec3b(0, 255, 255);
            }

            // 为了上位机看得清，放大到 160x120
            cv::resize(out_bgr, out_bgr, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);
            return true;
        }
    #endif

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
                              uint32_t loop_delay_us,
                              bool enable_udp_stream,
                              const char *udp_target_ip,
                              uint16_t udp_target_port,
                              int udp_jpeg_quality,
                              uint32_t udp_send_interval_ms)
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

    lq_udp_client udp_client;
    bool udp_ready = false;
    auto last_udp_send_tp = std::chrono::steady_clock::now();

    if (enable_udp_stream)
    {
        udp_client.udp_client_init(udp_target_ip, udp_target_port);
        udp_ready = (udp_client.get_udp_socket_fd() >= 0);
        if (!udp_ready)
        {
            lq_log_error("UDP stream enabled but init failed");
        }
        else
        {
            lq_log_info("UDP stream enabled: %s:%u", udp_target_ip, udp_target_port);
        }
    }

    while (ls_system_running.load())
    {
        if (!CarRuntime_ProcessFrame(cam.get_raw_frame(), enable_motor))
        {
            usleep(empty_frame_delay_us);
            continue;
        }

        if (udp_ready)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_udp_send_tp).count();
            if (elapsed_ms >= (long long)udp_send_interval_ms)
            {
                cv::Mat overlay_bgr;
                if (CarRuntime_BuildGrayOverlayFrame(overlay_bgr))
                {
                    if (udp_client.udp_send_image(overlay_bgr, udp_jpeg_quality) < 0)
                    {
                        lq_log_error("UDP send overlay image failed");
                    }
                }
                last_udp_send_tp = now;
            }
        }        

        usleep(loop_delay_us);
    }

    CarRuntime_Shutdown(enable_motor);

    return true;
#endif
}
