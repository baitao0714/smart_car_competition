#include "car_runtime.hpp"
#include "image.hpp"
#include "lq_common.hpp"
#include <algorithm>
#include <chrono>
#ifdef LQ_HAVE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

#ifdef LQ_HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

namespace {
#ifdef LQ_HAVE_OPENCV
static bool CarRuntime_BuildGrayOverlayFrame(cv::Mat& out_bgr) {
	// 使用原始的彩色缩放图作为 overlay 底图，而不是二值图。
	// 这样上位机看到的是原图（彩色），但巡线处理仍使用二值化数据不变。
	if (Resized_image.empty()) {
		return false;
	}
	// 不修改全局 Resized_image，复制到 out_bgr
	out_bgr = Resized_image.clone();

	const int rows = out_bgr.rows;
	const int cols = out_bgr.cols;
	const int off_line =
	    std::max(0, std::min((int)ImageStatus.OFFLine, rows - 1));

	for (int y = rows - 1; y > off_line; --y) {
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

	// 在 overlay 上绘制 ElementDetect 的 ROI（如果存在），把 First_image
	// 坐标映射到当前 out_bgr
	if (ElementDetect.enable && ElementDetect.red_found &&
	    !First_image.empty()) {
		const int cut_width = CropCutWidth;
		const int cut_height = CropCutHeight;
		int cut_x = std::max(0, (First_image.cols - cut_width) / 2);
		int cut_y = std::max(0, (First_image.rows - cut_height) / 2);
		float sx =
		    static_cast<float>(out_bgr.cols) / static_cast<float>(cut_width);
		float sy =
		    static_cast<float>(out_bgr.rows) / static_cast<float>(cut_height);

		// marker (red)
		int mx = ElementDetect.red_x - cut_x;
		int my = ElementDetect.red_y - cut_y;
		int mw = ElementDetect.red_w;
		int mh = ElementDetect.red_h;
		int rx = static_cast<int>(std::round(mx * sx));
		int ry = static_cast<int>(std::round(my * sy));
		int rw = std::max(1, static_cast<int>(std::round(mw * sx)));
		int rh = std::max(1, static_cast<int>(std::round(mh * sy)));
		cv::Rect rrect(rx, ry, rw, rh);
		cv::Rect bound(0, 0, out_bgr.cols, out_bgr.rows);
		rrect &= bound;
		if (rrect.area() > 0) {
			cv::rectangle(out_bgr, rrect, cv::Scalar(0, 0, 255), 1); // red
		}

		// classify ROI (green)
		int cx = ElementDetect.roi_x - cut_x;
		int cy = ElementDetect.roi_y - cut_y;
		int cw = ElementDetect.roi_w;
		int ch = ElementDetect.roi_h;
		int crx = static_cast<int>(std::round(cx * sx));
		int cry = static_cast<int>(std::round(cy * sy));
		int crw = std::max(1, static_cast<int>(std::round(cw * sx)));
		int crh = std::max(1, static_cast<int>(std::round(ch * sy)));
		cv::Rect crect(crx, cry, crw, crh);
		crect &= bound;
		if (crect.area() > 0) {
			cv::rectangle(out_bgr, crect, cv::Scalar(0, 255, 0), 1); // green
		}
	}

	cv::resize(out_bgr, out_bgr, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);
	return true;
}
#endif

bool car_runtime_initialized = false;
bool car_runtime_motor_enabled = false;
int car_runtime_motor_init_duty = 1000;
} // namespace

void CarRuntime_Init(bool enable_motor, int motor_init_duty) {
	Data_Settings();
	car_runtime_motor_enabled = enable_motor;
	car_runtime_motor_init_duty = motor_init_duty;

	if (enable_motor) {
		Motor_Init1(motor_init_duty);
		Motor_Argument();
	}

	car_runtime_initialized = true;
}

void CarRuntime_Shutdown(bool disable_motor) {
	if (disable_motor && car_runtime_motor_enabled) {
		Motor_Disable1();
	}

	cleanup();
	car_runtime_initialized = false;
}

#ifdef LQ_HAVE_OPENCV
bool CarRuntime_ProcessFrame(const cv::Mat& frame, bool enable_motor) {
	if (!car_runtime_initialized || car_runtime_motor_enabled != enable_motor) {
		CarRuntime_Init(enable_motor, car_runtime_motor_init_duty);
	}

	First_image = frame;
	if (First_image.empty()) {
		return false;
	}

	ImageProcess();
	if (enable_motor) {
		Motor_Control();
	}

	return true;
}
#endif

bool CarRuntime_RunCameraLoop(
    bool enable_motor, uint16_t width, uint16_t height, uint16_t fps,
    int motor_init_duty, uint32_t empty_frame_delay_us, uint32_t loop_delay_us,
    bool enable_udp_stream, const char* udp_target_ip, uint16_t udp_target_port,
    int udp_jpeg_quality, uint32_t udp_send_interval_ms) {
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
	lq_camera_ex cam(width, height, fps);
	if (!cam.is_cam_opened()) {
		lq_log_error("Failed to open camera for car runtime");
		return false;
	}

	CarRuntime_Init(enable_motor, motor_init_duty);

	lq_udp_client
	    udp_client_raw; // send original camera image -> 192.168.31.187
	lq_udp_client
	    udp_client_crop; // send cropped & binarized image -> 192.168.31.96
	bool udp_ready_raw = false;
	bool udp_ready_crop = false;
	auto last_udp_send_tp = std::chrono::steady_clock::now();

	if (enable_udp_stream) {
		// raw camera image target
		const char* RAW_TARGET_IP = "192.168.31.187";
		// cropped + binarized image target
		const char* CROP_TARGET_IP = "192.168.31.96";

		udp_client_raw.udp_client_init(RAW_TARGET_IP, udp_target_port);
		udp_ready_raw = (udp_client_raw.get_udp_socket_fd() >= 0);
		if (!udp_ready_raw) {
			lq_log_error("UDP raw stream init failed: %s:%u", RAW_TARGET_IP,
			             udp_target_port);
		} else {
			lq_log_info("UDP raw stream enabled: %s:%u", RAW_TARGET_IP,
			            udp_target_port);
		}

		udp_client_crop.udp_client_init(CROP_TARGET_IP, udp_target_port);
		udp_ready_crop = (udp_client_crop.get_udp_socket_fd() >= 0);
		if (!udp_ready_crop) {
			lq_log_error("UDP crop stream init failed: %s:%u", CROP_TARGET_IP,
			             udp_target_port);
		} else {
			lq_log_info("UDP crop stream enabled: %s:%u", CROP_TARGET_IP,
			            udp_target_port);
		}
	}

	while (ls_system_running.load()) {
		cv::Mat frame = cam.get_frame_raw();
		if (frame.empty()) {
			usleep(empty_frame_delay_us);
			continue;
		}

		if (!CarRuntime_ProcessFrame(frame, enable_motor)) {
			usleep(empty_frame_delay_us);
			continue;
		}

		if (udp_ready_raw || udp_ready_crop) {
			auto now = std::chrono::steady_clock::now();
			auto elapsed_ms =
			    std::chrono::duration_cast<std::chrono::milliseconds>(
			        now - last_udp_send_tp)
			        .count();
			if (elapsed_ms >= (long long)udp_send_interval_ms) {
				// 1) send raw camera frame to RAW target (with ROI overlay)
				if (udp_ready_raw) {
					cv::Mat raw_vis = frame.clone();
					if (ElementDetect.enable && ElementDetect.red_found) {
						// marker (red)
						cv::Rect mrect(ElementDetect.red_x, ElementDetect.red_y,
						               ElementDetect.red_w,
						               ElementDetect.red_h);
						cv::Rect bound_raw(0, 0, raw_vis.cols, raw_vis.rows);
						mrect &= bound_raw;
						if (mrect.area() > 0)
							cv::rectangle(raw_vis, mrect, cv::Scalar(0, 0, 255),
							              2);

						// classify ROI (green)
						cv::Rect rrect(ElementDetect.roi_x, ElementDetect.roi_y,
						               ElementDetect.roi_w,
						               ElementDetect.roi_h);
						rrect &= bound_raw;
						if (rrect.area() > 0)
							cv::rectangle(raw_vis, rrect, cv::Scalar(0, 255, 0),
							              2);
					}

					if (udp_client_raw.udp_send_image(raw_vis,
					                                  udp_jpeg_quality) < 0) {
						lq_log_error("UDP send raw image failed");
					}
				}

				// 2) send cropped & binarized image to CROP target (showing
				// line results)
				if (udp_ready_crop) {
					cv::Mat bin_vis;
					// Prefer Gray_image (resized to LCDW x LCDH)
					if (!Gray_image.empty()) {
						cv::Mat bin_img;
						cv::threshold(Gray_image, bin_img, 0, 255,
						              cv::THRESH_BINARY | cv::THRESH_OTSU);
						cv::cvtColor(bin_img, bin_vis, cv::COLOR_GRAY2BGR);

						// draw line overlays onto bin_vis using ImageDeal
						const int rows = bin_vis.rows;
						const int cols = bin_vis.cols;
						const int off_line = std::max(
						    0, std::min((int)ImageStatus.OFFLine, rows - 1));
						for (int y = rows - 1; y > off_line; --y) {
							int l = std::max(
							    0, std::min((int)ImageDeal[y].LeftBorder,
							                cols - 1));
							int r = std::max(
							    0, std::min((int)ImageDeal[y].RightBorder,
							                cols - 1));
							int c =
							    std::max(0, std::min((int)ImageDeal[y].Center,
							                         cols - 1));
							bin_vis.at<cv::Vec3b>(y, l) = cv::Vec3b(0, 255, 0);
							bin_vis.at<cv::Vec3b>(y, r) = cv::Vec3b(0, 0, 255);
							bin_vis.at<cv::Vec3b>(y, c) =
							    cv::Vec3b(0, 255, 255);
						}

						// draw ElementDetect ROI mapped from First_image to
						// bin_vis
						if (ElementDetect.enable && ElementDetect.red_found &&
						    !First_image.empty()) {
							const int cut_width = CropCutWidth;
							const int cut_height = CropCutHeight;
							int cut_x =
							    std::max(0, (First_image.cols - cut_width) / 2);
							int cut_y = std::max(
							    0, (First_image.rows - cut_height) / 2);
							float sx = static_cast<float>(bin_vis.cols) /
							           static_cast<float>(cut_width);
							float sy = static_cast<float>(bin_vis.rows) /
							           static_cast<float>(cut_height);

							int mx = ElementDetect.red_x - cut_x;
							int my = ElementDetect.red_y - cut_y;
							int mw = ElementDetect.red_w;
							int mh = ElementDetect.red_h;
							int rx = static_cast<int>(std::round(mx * sx));
							int ry = static_cast<int>(std::round(my * sy));
							int rw = std::max(
							    1, static_cast<int>(std::round(mw * sx)));
							int rh = std::max(
							    1, static_cast<int>(std::round(mh * sy)));
							cv::Rect rrect(rx, ry, rw, rh);
							cv::Rect bound(0, 0, bin_vis.cols, bin_vis.rows);
							rrect &= bound;
							if (rrect.area() > 0)
								cv::rectangle(bin_vis, rrect,
								              cv::Scalar(0, 0, 255), 1);

							int cx = ElementDetect.roi_x - cut_x;
							int cy = ElementDetect.roi_y - cut_y;
							int cw = ElementDetect.roi_w;
							int ch = ElementDetect.roi_h;
							int crx = static_cast<int>(std::round(cx * sx));
							int cry = static_cast<int>(std::round(cy * sy));
							int crw = std::max(
							    1, static_cast<int>(std::round(cw * sx)));
							int crh = std::max(
							    1, static_cast<int>(std::round(ch * sy)));
							cv::Rect crect(crx, cry, crw, crh);
							crect &= bound;
							if (crect.area() > 0)
								cv::rectangle(bin_vis, crect,
								              cv::Scalar(0, 255, 0), 1);
						}

						cv::resize(bin_vis, bin_vis, cv::Size(160, 120), 0, 0,
						           cv::INTER_NEAREST);
						if (udp_client_crop.udp_send_image(
						        bin_vis, udp_jpeg_quality) < 0) {
							lq_log_error("UDP send crop(bin) image failed");
						}
					} else if (!Resized_image.empty()) {
						// fallback: build from Resized_image
						cv::Mat gray_tmp;
						cv::cvtColor(Resized_image, gray_tmp,
						             cv::COLOR_BGR2GRAY);
						cv::Mat bin_img;
						cv::threshold(gray_tmp, bin_img, 0, 255,
						              cv::THRESH_BINARY | cv::THRESH_OTSU);
						cv::cvtColor(bin_img, bin_vis, cv::COLOR_GRAY2BGR);

						const int rows = bin_vis.rows;
						const int cols = bin_vis.cols;
						const int off_line = std::max(
						    0, std::min((int)ImageStatus.OFFLine, rows - 1));
						for (int y = rows - 1; y > off_line; --y) {
							int l = std::max(
							    0, std::min((int)ImageDeal[y].LeftBorder,
							                cols - 1));
							int r = std::max(
							    0, std::min((int)ImageDeal[y].RightBorder,
							                cols - 1));
							int c =
							    std::max(0, std::min((int)ImageDeal[y].Center,
							                         cols - 1));
							bin_vis.at<cv::Vec3b>(y, l) = cv::Vec3b(0, 255, 0);
							bin_vis.at<cv::Vec3b>(y, r) = cv::Vec3b(0, 0, 255);
							bin_vis.at<cv::Vec3b>(y, c) =
							    cv::Vec3b(0, 255, 255);
						}

						cv::resize(bin_vis, bin_vis, cv::Size(160, 120), 0, 0,
						           cv::INTER_NEAREST);
						if (udp_client_crop.udp_send_image(
						        bin_vis, udp_jpeg_quality) < 0) {
							lq_log_error("UDP send crop(bin) image failed");
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

// extra closing brace to balance edits
}
