#ifndef _LQ_CAMERA_HPP
#define _LQ_CAMERA_HPP

#include "lq_camera_ex.hpp"

// 兼容层: 让旧代码中使用的 `lq_camera` API 继续工作，底层使用 `lq_camera_ex`
#ifdef LQ_HAVE_OPENCV

class lq_camera
{
public:
    lq_camera(uint16_t _width, uint16_t _height, uint16_t _fps, int /*_dev_id*/ = 0)
        : impl(nullptr)
    {
        impl = std::make_unique<lq_camera_ex>(_width, _height, _fps);
        impl->init(_width, _height, _fps);
        impl->start_collect();
    }

    lq_camera(const lq_camera&) = delete;
    lq_camera& operator=(const lq_camera&) = delete;

    ~lq_camera()
    {
        if (impl)
        {
            impl->stop_collect();
        }
    }

public:
    bool open(uint16_t _width, uint16_t _height, uint16_t _fps)
    {
        if (!impl)
            impl = std::make_unique<lq_camera_ex>(_width, _height, _fps);
        int rc = impl->init(_width, _height, _fps);
        if (rc == 0)
            impl->start_collect();
        return rc == 0;
    }

    void close()
    {
        if (impl)
        {
            impl->stop_collect();
        }
    }

    cv::Mat get_raw_frame()
    {
        cv::Mat mat;
        if (impl)
        {
            impl->get_frame(mat, true);
        }
        return mat;
    }

    bool is_opened() const
    {
        return impl ? impl->is_cam_opened() : false;
    }

    uint16_t get_height() const { return impl ? impl->get_camera_height() : 0; }
    uint16_t get_width() const  { return impl ? impl->get_camera_width() : 0; }
    uint16_t get_fps() const    { return impl ? impl->get_camera_fps() : 0; }
    int      get_dev_id() const { return 0; }

private:
    std::unique_ptr<lq_camera_ex> impl;
};

#else

// 非 OpenCV 情况的占位实现，避免编译失败
class lq_camera
{
public:
    lq_camera(uint16_t, uint16_t, uint16_t, int = 0) {}
    ~lq_camera() {}
    bool open(uint16_t, uint16_t, uint16_t) { return false; }
    void close() {}
    bool is_opened() const { return false; }
    uint16_t get_height() const { return 0; }
    uint16_t get_width() const { return 0; }
    uint16_t get_fps() const { return 0; }
    int get_dev_id() const { return -1; }
    cv::Mat get_raw_frame() { return cv::Mat(); }
};

#endif

#endif