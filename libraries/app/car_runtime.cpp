#include "car_runtime.hpp"
#include "image.hpp"
#include "lq_common.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
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

			// 叠加检测类别标签
			const char* cls_label = "detecting...";
			if (ElementDetect.stable_frames >=
			    ElementDetect.stable_frames_need) {
				switch (ElementDetect.class_id) {
				case 0:
					cls_label = "supplies";
					break;
				case 1:
					cls_label = "vehicle";
					break;
				case 2:
					cls_label = "weapon";
					break;
				default:
					cls_label = "unknown";
					break;
				}
			}
			int label_x = crx;
			int label_y = cry - 6;
			if (label_y < 10)
				label_y = cry + crh + 14;
			cv::putText(out_bgr, cls_label, cv::Point(label_x, label_y),
			            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0),
			            1);
		}
	}

	cv::resize(out_bgr, out_bgr, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);
	return true;
}
#endif

bool car_runtime_initialized = false;
bool car_runtime_motor_enabled = false;
int car_runtime_motor_init_duty = 1000;

// ====== K0 按键使能位 ======
// K0 按下(PIN_44 低电平)后，小车才开始运行
static bool g_car_enabled_by_key = false;
static std::unique_ptr<ls_gpio> g_k0_button;

// ====== NCNN 检测绕行状态 ======
// detour_mode: 0=正常循中线, 1=武器左绕(跟左线), 2=物资右绕(跟右线)
static int g_detour_mode = 0;
static std::chrono::steady_clock::time_point g_detour_start;
static std::chrono::steady_clock::time_point g_straight_start;
static int g_detour_cooldown = 600;          // 冷却帧数，防止重复触发
static const int kDetourDurationMs = 2000;   // 绕行持续时间(毫秒)
static const int kStraightDurationMs = 1000; // 直行压过持续时间(毫秒)
static const int kDetourCooldown = 60;       // 绕行结束后的冷却帧数
static const int kDetourSpeedDelta = 30;     // 绕行时速度减小量
static const float kDetourKpDelta = 2.0f;    // 绕行时差速P减小量

static void ApplyDetourControl() {
	auto now = std::chrono::steady_clock::now();

	// 只在非环岛、非特殊路段时工作（绕行激活期间不受此限制）
	bool detour_active = (g_detour_mode != 0);
	bool straight_active = (g_straight_start.time_since_epoch().count() > 0);
	if (!detour_active && !straight_active) {
		if (ImageStatus.Road_type == LeftCirque ||
		    ImageStatus.Road_type == RightCirque) {
			return;
		}
	}

	// 冷却递减
	if (g_detour_cooldown > 0) {
		g_detour_cooldown--;
	}

	// 检测到新元素且不在绕行/冷却中
	if (ElementDetect.route_mode != 0 && !detour_active && !straight_active &&
	    g_detour_cooldown <= 0) {

		printf("DETOUR-TRIG: cooldown=%d route_mode=%d class_id=%d\n",
		       g_detour_cooldown, ElementDetect.route_mode,
		       ElementDetect.class_id);
		int class_id = ElementDetect.class_id;
		std::printf("Detour: element class=%d route_mode=%d\n", class_id,
		            ElementDetect.route_mode);

		if (class_id == 2) {
			// weapon: 左侧绕行，跟左线
			g_detour_mode = 1;
			g_detour_start = now;
			Speed_Goal_l -= kDetourSpeedDelta;
			Speed_Goal_r -= kDetourSpeedDelta;
			Diff_Kp -= kDetourKpDelta;
			std::printf("Detour: weapon -> follow LEFT line for %d ms\n",
			            kDetourDurationMs);
		} else if (class_id == 0) {
			// supplies: 右侧绕行，跟右线
			g_detour_mode = 2;
			g_detour_start = now;
			Speed_Goal_l -= kDetourSpeedDelta;
			Speed_Goal_r -= kDetourSpeedDelta;
			Diff_Kp -= kDetourKpDelta;
			std::printf("Detour: supplies -> follow RIGHT line for %d ms\n",
			            kDetourDurationMs);
		} else if (class_id == 1) {
			// vehicle: 直行压过
			g_straight_start = now;
			std::printf("Detour: vehicle -> straight pass for %d ms\n",
			            kStraightDurationMs);
		}
	}

	// 车辆模式：直行，不做中线偏移（正常循中线）
	if (straight_active) {
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
		                   now - g_straight_start)
		                   .count();
		if (elapsed >= kStraightDurationMs) {
			g_straight_start = std::chrono::steady_clock::time_point{};
			g_detour_cooldown = kDetourCooldown;
			std::printf("Detour: vehicle straight pass finished, cooldown=%d\n",
			            kDetourCooldown);
		}
		return;
	}

	// 绕行模式：偏离中线
	if (detour_active) {
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
		                   now - g_detour_start)
		                   .count();

		if (elapsed >= kDetourDurationMs) {
			std::printf(
			    "Detour: finished after %lld ms, back to normal, cooldown=%d\n",
			    (long long)elapsed, kDetourCooldown);
			g_detour_mode = 0;
			g_detour_cooldown = kDetourCooldown;
			Speed_Goal_l += kDetourSpeedDelta;
			Speed_Goal_r += kDetourSpeedDelta;
			Diff_Kp += kDetourKpDelta;
			ElementDetect.route_mode = 0;
			ElementDetect.class_id = 0;
			SystemData.Model = 0;
		} else {
			// 绕行期间：基于实际边界计算中线
			int tp = ImageStatus.TowPoint_True;
			if (tp < ImageStatus.OFFLine + 1)
				tp = ImageStatus.OFFLine + 1;
			if (tp > 49)
				tp = 49;
			if (g_detour_mode == 1) {
				// weapon: 跟左线
				ImageDeal[tp].Center = ImageDeal[tp].LeftBorder + 5;
			} else {
				// supplies: 跟右线
				ImageDeal[tp].Center = ImageDeal[tp].RightBorder - 5;
			}
			// 逐行更新中线
			for (int y = 59; y > ImageStatus.OFFLine; y--) {
				if (g_detour_mode == 1)
					ImageDeal[y].Center = ImageDeal[y].LeftBorder + 5;
				else
					ImageDeal[y].Center = ImageDeal[y].RightBorder - 5;
			}

			GetDet();
			// 每秒打印一次绕行状态
			static auto last_detour_log = std::chrono::steady_clock::now();
			if (elapsed > 0 && elapsed % 1000 < 50) {
				last_detour_log = now;
				std::printf("Detour[%lldms]: mode=%d L=%d R=%d C=%d\n",
				            (long long)elapsed, g_detour_mode,
				            ImageDeal[tp].LeftBorder, ImageDeal[tp].RightBorder,
				            ImageDeal[tp].Center);
			}
		}
	}
}
} // namespace

void CarRuntime_Init(bool enable_motor, int motor_init_duty) {
	Data_Settings();
	car_runtime_motor_enabled = enable_motor;
	car_runtime_motor_init_duty = motor_init_duty;

	// 初始化 K0 按键 GPIO (PIN_44, 输入模式)
	g_k0_button = std::make_unique<ls_gpio>(PIN_44, GPIO_MODE_IN);
	g_car_enabled_by_key = false;

	if (enable_motor) {
		Motor_Init1(motor_init_duty);
		Motor_Argument();
	}

	g_detour_mode = 0;
	g_detour_start = std::chrono::steady_clock::time_point{};
	g_straight_start = std::chrono::steady_clock::time_point{};
	g_detour_cooldown = 0;

	car_runtime_initialized = true;
}

void CarRuntime_Shutdown(bool disable_motor) {
	if (disable_motor && car_runtime_motor_enabled) {
		Motor_Disable1();
	}

	g_k0_button.reset();
	g_car_enabled_by_key = false;

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

	// 应用 NCNN 检测结果的绕行控制
	ApplyDetourControl();

	// K0 按键使能检测：按下(PIN_44低电平)后小车才开始运行
	if (!g_car_enabled_by_key && g_k0_button) {
		if (g_k0_button->gpio_level_get() == GPIO_LOW) {
			g_car_enabled_by_key = true;
			printf("K0 button pressed, car enabled!\n");
		}
	}

	if (enable_motor && g_car_enabled_by_key) {
		Motor_Control();
	}

	return true;
}
#endif

bool CarRuntime_RunCameraLoop(
    bool enable_motor, uint16_t width, uint16_t height, uint16_t fps,
    int motor_init_duty, uint32_t empty_frame_delay_us, uint32_t loop_delay_us,
    bool enable_udp_stream, const char* udp_target_ip,
    const char* udp_crop_target_ip, uint16_t udp_target_port,
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
		const char* RAW_TARGET_IP = udp_target_ip;
		// cropped + binarized image target
		const char* CROP_TARGET_IP = udp_crop_target_ip;

		udp_client_raw.udp_client_init(RAW_TARGET_IP, udp_target_port);
		udp_ready_raw = false; // disabled
		if (!udp_ready_raw) {
			lq_log_error("UDP raw stream init failed: %s:%u", RAW_TARGET_IP,
			             udp_target_port);
		} else {
			lq_log_info("UDP raw stream enabled: %s:%u", RAW_TARGET_IP,
			            udp_target_port);
		}

		udp_client_crop.udp_client_init(CROP_TARGET_IP, udp_target_port);
		// udp_ready_crop = (udp_client_crop.get_udp_socket_fd() >= 0);
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
						if (rrect.area() > 0) {
							cv::rectangle(raw_vis, rrect, cv::Scalar(0, 255, 0),
							              2);

							// 叠加检测类别标签
							const char* cls_label = "detecting...";
							if (ElementDetect.stable_frames >=
							    ElementDetect.stable_frames_need) {
								switch (ElementDetect.class_id) {
								case 0:
									cls_label = "supplies";
									break;
								case 1:
									cls_label = "vehicle";
									break;
								case 2:
									cls_label = "weapon";
									break;
								default:
									cls_label = "unknown";
									break;
								}
							}
							int label_x = rrect.x;
							int label_y = rrect.y - 8;
							if (label_y < 12)
								label_y = rrect.y + rrect.height + 16;
							cv::putText(raw_vis, cls_label,
							            cv::Point(label_x, label_y),
							            cv::FONT_HERSHEY_SIMPLEX, 0.5,
							            cv::Scalar(0, 255, 0), 2);
						}
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
							bin_vis.at<cv::Vec3b>(y, l) =
							    (ImageDeal[y].IsLeftFind == 'T')
							        ? cv::Vec3b(0, 255, 0)
							        : cv::Vec3b(255, 0, 0);
							bin_vis.at<cv::Vec3b>(y, r) =
							    (ImageDeal[y].IsRightFind == 'T')
							        ? cv::Vec3b(0, 0, 255)
							        : cv::Vec3b(255, 0, 0);
							bin_vis.at<cv::Vec3b>(y, c) =
							    (ImageFlag.image_element_rings == 0)
							        ? cv::Vec3b(0, 255, 255)
							        : cv::Vec3b(255, 0, 0);
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
							bin_vis.at<cv::Vec3b>(y, l) =
							    (ImageDeal[y].IsLeftFind == 'T')
							        ? cv::Vec3b(0, 255, 0)
							        : cv::Vec3b(255, 0, 0);
							bin_vis.at<cv::Vec3b>(y, r) =
							    (ImageDeal[y].IsRightFind == 'T')
							        ? cv::Vec3b(0, 0, 255)
							        : cv::Vec3b(255, 0, 0);
							bin_vis.at<cv::Vec3b>(y, c) =
							    (ImageFlag.image_element_rings == 0)
							        ? cv::Vec3b(0, 255, 255)
							        : cv::Vec3b(255, 0, 0);
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

	} // close while

	CarRuntime_Shutdown(enable_motor);

	return true;
#endif
}
