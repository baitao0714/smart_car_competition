#include "motor.hpp"
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <sys/stat.h>

namespace {
constexpr pwm_pin_t kLeftMotorPwmPin = PWM1_PIN65;
constexpr pwm_pin_t kRightMotorPwmPin = PWM2_PIN66;
constexpr gpio_pin_t kLeftMotorDirPin = PIN_75;
constexpr gpio_pin_t kRightMotorDirPin = PIN_74;
constexpr ls_enc_pwm_pin_t kLeftEncoderPin = ENC_PWM3_PIN67;
constexpr ls_enc_pwm_pin_t kRightEncoderPin = ENC_PWM0_PIN64;
constexpr gpio_pin_t kLeftEncoderDirPin = PIN_72;
constexpr gpio_pin_t kRightEncoderDirPin = PIN_73;
constexpr uint32_t kMotorPwmFreqHz = 10000;
constexpr int kDefaultInitDuty = 1000;
// 正向（小车前进方向）时，方向 GPIO 应输出的电平
constexpr bool kLeftForwardDir = true;
constexpr bool kRightForwardDir = true;

std::unique_ptr<ls_pwm> left_motor_pwm;
std::unique_ptr<ls_pwm> right_motor_pwm;
std::unique_ptr<ls_gpio> left_motor_dir;
std::unique_ptr<ls_gpio> right_motor_dir;
std::unique_ptr<ls_encoder_pwm> left_motor_encoder;
std::unique_ptr<ls_encoder_pwm> right_motor_encoder;

bool motor_initialized = false;

int clamp_pwm(int value) {
	if (value < 0) {
		return 0;
	}
	if (value > PWM_DUTY_MAX) {
		return PWM_DUTY_MAX;
	}
	return value;
}

int clamp_pid_output(int value, int min_value, int max_value) {
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

void ensure_motor_ready() {
	if (!motor_initialized) {
		Motor_Init1(kDefaultInitDuty);
	}
}
} // namespace

int16_t encoder_Left = 0;
int16_t encoder_Right = 0;
int16_t Speed_Encoder_l = 0;
float Speed_P_l = 0;
float Speed_I_l = 0;
float Speed_D_l = 0;
int Speed_Erro_l = 0;
int Speed_Goal_l = 0;
int Speed_PID_OUT_l = 0;
int Speed_Lasterro_l = 0;
int Speed_Preverro_l = 0;

int16_t Speed_Encoder_r = 0;
float Speed_P_r = 0;
float Speed_I_r = 0;
float Speed_D_r = 0;
int Speed_Erro_r = 0;
int Speed_Goal_r = 0;
int Speed_PID_OUT_r = 0;
int Speed_Lasterro_r = 0;
int Speed_Preverro_r = 0;

int PWM_Max = 4000;
int PWM_Min = 50; //-5000
int16_t Speed_Begin = 80;
int16_t Speed_Expect = 0;
float Diff_Speed_error = 0;
int16_t Diff_SpeedL_expect = 0;
int16_t Diff_SpeedR_expect = 0;

float Diff_Kp = 10.242f; // 10.242
float Diff_Kd = 20.274f;
uint8_t stop_flag = 0;

// ========== PID 热加载配置 ==========
namespace {
static time_t g_pid_cfg_mtime = 0;
static const char* kPidConfigPath = "pid.conf";

static const float kDefaultSpeedPL = 6.5f;
static const float kDefaultSpeedIL = 0.0f;
static const float kDefaultSpeedDL = 0.0f;
static const float kDefaultSpeedPR = 6.5f;
static const float kDefaultSpeedIR = 0.0f;
static const float kDefaultSpeedDR = 0.0f;
static const float kDefaultDiffKp = 10.242f;
static const float kDefaultDiffKd = 20.274f;

static std::string Trim(const std::string& s) {
	const char* ws = " \t\r\n";
	auto start = s.find_first_not_of(ws);
	if (start == std::string::npos)
		return std::string();
	auto end = s.find_last_not_of(ws);
	return s.substr(start, end - start + 1);
}

void LoadPidConfigImpl(const char* path) {
	std::ifstream ifs(path);
	if (!ifs.is_open()) {
		// 创建默认配置文件
		std::ofstream ofs(path);
		if (!ofs.is_open())
			return;
		ofs << "# pid.conf - PID hot-load config\n";
		ofs << "# speed_p_l/speed_i_l/speed_d_l: left motor speed PID\n";
		ofs << "# speed_p_r/speed_i_r/speed_d_r: right motor speed PID\n";
		ofs << "# diff_kp/diff_kd: differential PD params\n";
		ofs << "speed_p_l=6.5\n";
		ofs << "speed_i_l=0.0\n";
		ofs << "speed_d_l=0.0\n";
		ofs << "speed_p_r=6.5\n";
		ofs << "speed_i_r=0.0\n";
		ofs << "speed_d_r=0.0\n";
		ofs << "diff_kp=10.242\n";
		ofs << "diff_kd=20.274\n";
		ofs.close();
		Speed_P_l = kDefaultSpeedPL;
		Speed_I_l = kDefaultSpeedIL;
		Speed_D_l = kDefaultSpeedDL;
		Speed_P_r = kDefaultSpeedPR;
		Speed_I_r = kDefaultSpeedIR;
		Speed_D_r = kDefaultSpeedDR;
		Diff_Kp = kDefaultDiffKp;
		Diff_Kd = kDefaultDiffKd;
		return;
	}

	std::string line;
	float spl = kDefaultSpeedPL;
	float sil = kDefaultSpeedIL;
	float sdl = kDefaultSpeedDL;
	float spr = kDefaultSpeedPR;
	float sir = kDefaultSpeedIR;
	float sdr = kDefaultSpeedDR;
	float dkp = kDefaultDiffKp;
	float dkd = kDefaultDiffKd;

	while (std::getline(ifs, line)) {
		line = Trim(line);
		if (line.empty() || line[0] == '#')
			continue;
		auto pos = line.find('=');
		if (pos == std::string::npos)
			continue;
		auto key = Trim(line.substr(0, pos));
		auto val = Trim(line.substr(pos + 1));
		float fv = 0.0f;
		try {
			fv = std::stof(val);
		} catch (...) {
			continue;
		}
		if (key == "speed_p_l")
			spl = fv;
		else if (key == "speed_i_l")
			sil = fv;
		else if (key == "speed_d_l")
			sdl = fv;
		else if (key == "speed_p_r")
			spr = fv;
		else if (key == "speed_i_r")
			sir = fv;
		else if (key == "speed_d_r")
			sdr = fv;
		else if (key == "diff_kp")
			dkp = fv;
		else if (key == "diff_kd")
			dkd = fv;
	}
	ifs.close();

	Speed_P_l = spl;
	Speed_I_l = sil;
	Speed_D_l = sdl;
	Speed_P_r = spr;
	Speed_I_r = sir;
	Speed_D_r = sdr;
	Diff_Kp = dkp;
	Diff_Kd = dkd;

	struct stat st;
	if (stat(path, &st) == 0)
		g_pid_cfg_mtime = st.st_mtime;
	std::printf("Loaded pid config from %s: P_l=%.3f I_l=%.3f D_l=%.3f "
	            "P_r=%.3f I_r=%.3f D_r=%.3f Kp=%.3f Kd=%.3f\n",
	            path, Speed_P_l, Speed_I_l, Speed_D_l,
	            Speed_P_r, Speed_I_r, Speed_D_r,
	            Diff_Kp, Diff_Kd);
}

void PidConfigReloadIfNeededImpl(const char* path) {
	struct stat st;
	if (stat(path, &st) != 0) {
		LoadPidConfig(path);
		return;
	}
	if (st.st_mtime != g_pid_cfg_mtime) {
		LoadPidConfig(path);
	}
}
} // namespace

void LoadPidConfig(const char* path) { LoadPidConfigImpl(path); }

void PidConfigReloadIfNeeded(const char* path) {
	PidConfigReloadIfNeededImpl(path);
}
// ========== PID 热加载配置结束 ==========

float Encoder_Left1(void) {
	ensure_motor_ready();
	encoder_Left =
	    -static_cast<int16_t>(left_motor_encoder->encoder_get_count());
	return encoder_Left;
}

float Encoder_Right1(void) {
	ensure_motor_ready();
	encoder_Right =
	    static_cast<int16_t>(right_motor_encoder->encoder_get_count());
	return encoder_Right;
}

void Encoder_Test1(void) {
	ensure_motor_ready();
	(void)Encoder_Left1();
	(void)Encoder_Right1();
}

void Motor_Argument(void) {
	Speed_Goal_l = 130;
	Speed_Goal_r = 130;

	Speed_P_l = 6.5f;
	Speed_I_l = 0; // 1.65f;
	Speed_D_l = 0;

	Speed_P_r = 6.5f;
	Speed_I_r = 0;
	Speed_D_r = 0;
}

void Motor_Init1(int duty) {
	const uint32_t init_duty = static_cast<uint32_t>(clamp_pwm(duty));

	left_motor_pwm = std::make_unique<ls_pwm>(kLeftMotorPwmPin, kMotorPwmFreqHz,
	                                          init_duty, PWM_POL_INV);
	right_motor_pwm = std::make_unique<ls_pwm>(
	    kRightMotorPwmPin, kMotorPwmFreqHz, init_duty, PWM_POL_INV);
	left_motor_dir = std::make_unique<ls_gpio>(kLeftMotorDirPin, GPIO_MODE_OUT);
	right_motor_dir =
	    std::make_unique<ls_gpio>(kRightMotorDirPin, GPIO_MODE_OUT);
	left_motor_encoder =
	    std::make_unique<ls_encoder_pwm>(kLeftEncoderPin, kLeftEncoderDirPin);
	right_motor_encoder =
	    std::make_unique<ls_encoder_pwm>(kRightEncoderPin, kRightEncoderDirPin);

	left_motor_dir->gpio_level_set(kLeftForwardDir ? GPIO_HIGH : GPIO_LOW);
	right_motor_dir->gpio_level_set(kRightForwardDir ? GPIO_HIGH : GPIO_LOW);
	left_motor_pwm->pwm_set_duty(0);
	right_motor_pwm->pwm_set_duty(0);

	encoder_Left = 0;
	encoder_Right = 0;
	Speed_Encoder_l = 0;
	Speed_Encoder_r = 0;
	Speed_Erro_l = 0;
	Speed_Erro_r = 0;
	Speed_PID_OUT_l = 0;
	Speed_PID_OUT_r = 0;
	Speed_Lasterro_l = 0;
	Speed_Lasterro_r = 0;
	Speed_Preverro_l = 0;
	Speed_Preverro_r = 0;
	Diff_SpeedL_expect = 0;
	Diff_SpeedR_expect = 0;
	motor_initialized = true;
}

void Left_Motor_Pwm1(int duty, bool dir) {
	ensure_motor_ready();
	left_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
	left_motor_pwm->pwm_set_duty(static_cast<uint32_t>(clamp_pwm(duty)));
}

void Right_Motor_Pwm1(int duty, bool dir) {
	ensure_motor_ready();
	right_motor_dir->gpio_level_set(dir ? GPIO_HIGH : GPIO_LOW);
	right_motor_pwm->pwm_set_duty(static_cast<uint32_t>(clamp_pwm(duty)));
}

void Motor_Disable1(void) {
	if (!motor_initialized) {
		return;
	}
	left_motor_pwm->pwm_set_duty(0);
	right_motor_pwm->pwm_set_duty(0);
	left_motor_pwm->pwm_disable();
	right_motor_pwm->pwm_disable();
	motor_initialized = false;
}

// ========== 修改点1：外环降频 ==========
void Motor_Control(void) {
	ensure_motor_ready();

	// 热加载 PID 参数配置
	PidConfigReloadIfNeeded(kPidConfigPath);

	static uint32_t debug_log_divider = 0;
	// [MOD] 外环分频计数器
	static uint8_t outer_loop_cnt = 0;

	// 读取编码器（内环每次都要用）
	encoder_Left =
	    -static_cast<int16_t>(left_motor_encoder->encoder_get_count());
	encoder_Right =
	    static_cast<int16_t>(right_motor_encoder->encoder_get_count());

	// 更新目标速度（根据 stop_flag）
	if (stop_flag == 1) {
		Speed_Goal_l = 0;
		Speed_Goal_r = 0;
	} else {
		Speed_Goal_l = 130;
		Speed_Goal_r = 130;

		if (top_point < 15) {
			Speed_Goal_l = 130;
			Speed_Goal_r = 130;
		} else {
			Speed_Goal_l = 130;
			Speed_Goal_r = 130;
		}
	}

	// [MOD] 外环每2次循环执行一次（降频为原来的1/2）
	outer_loop_cnt++;
	if (outer_loop_cnt >= 2) {
		outer_loop_cnt = 0;
		Motor_Diff_Pid1();   // 执行外环，更新 Diff_SpeedL_expect / Diff_SpeedR_expect
	}
	// 如果本次不执行外环，则 Diff_SpeedL_expect / Diff_SpeedR_expect 保持上一次的值

	// 内环每次执行
	Motor_PID_Left();
	Motor_PID_Right();

	if (++debug_log_divider >= 30) {
		debug_log_divider = 0;
		const int center_error = ImageStatus.Det_True - ImageStatus.MiddleLine;
		printf("err=%4d  pidL=%6d  pidR=%6d  spdL=%4d  spdR=%4d\n",
		       center_error, Speed_PID_OUT_l, Speed_PID_OUT_r, encoder_Left,
		       encoder_Right);
	}
}

// ========== 修改点2：左电机速度环加入积分冻结 ==========
void Motor_PID_Left(void) {
	ensure_motor_ready();

	Speed_Encoder_l = encoder_Left;
	Speed_Erro_l = Diff_SpeedL_expect - Speed_Encoder_l;

	// [MOD] 抗积分饱和（积分冻结）
	bool i_frozen = false;
	if ((Speed_PID_OUT_l >= PWM_Max && Speed_Erro_l > 0) ||
	    (Speed_PID_OUT_l <= -PWM_Max && Speed_Erro_l < 0)) {
		i_frozen = true;
	}

	if (!i_frozen) {
		// 正常累加完整 PID（含 I）
		Speed_PID_OUT_l += static_cast<int>(
		    Speed_P_l * (Speed_Erro_l - Speed_Lasterro_l) +
		    Speed_I_l * Speed_Erro_l +
		    Speed_D_l * (Speed_Erro_l - 2 * Speed_Lasterro_l + Speed_Preverro_l));
	} else {
		// 积分冻结：跳过 I 项
		Speed_PID_OUT_l += static_cast<int>(
		    Speed_P_l * (Speed_Erro_l - Speed_Lasterro_l) +
		    0 +  // 跳过积分项
		    Speed_D_l * (Speed_Erro_l - 2 * Speed_Lasterro_l + Speed_Preverro_l));
	}

	Speed_PID_OUT_l = clamp_pid_output(Speed_PID_OUT_l, PWM_Min, PWM_Max);

	Speed_Preverro_l = Speed_Lasterro_l;
	Speed_Lasterro_l = Speed_Erro_l;

	if (Speed_PID_OUT_l >= 0) {
		Left_Motor_Pwm1(Speed_PID_OUT_l, kLeftForwardDir);
	} else {
		Left_Motor_Pwm1(-Speed_PID_OUT_l, !kLeftForwardDir);
	}
}

// ========== 修改点2：右电机速度环加入积分冻结 ==========
void Motor_PID_Right(void) {
	ensure_motor_ready();

	Speed_Encoder_r = encoder_Right;
	Speed_Erro_r = Diff_SpeedR_expect - Speed_Encoder_r;

	// [MOD] 抗积分饱和（积分冻结）
	bool i_frozen = false;
	if ((Speed_PID_OUT_r >= PWM_Max && Speed_Erro_r > 0) ||
	    (Speed_PID_OUT_r <= -PWM_Max && Speed_Erro_r < 0)) {
		i_frozen = true;
	}

	if (!i_frozen) {
		// 正常累加完整 PID（含 I）
		Speed_PID_OUT_r += static_cast<int>(
		    Speed_P_r * (Speed_Erro_r - Speed_Lasterro_r) +
		    Speed_I_r * Speed_Erro_r +
		    Speed_D_r * (Speed_Erro_r - 2 * Speed_Lasterro_r + Speed_Preverro_r));
	} else {
		// 积分冻结：跳过 I 项
		Speed_PID_OUT_r += static_cast<int>(
		    Speed_P_r * (Speed_Erro_r - Speed_Lasterro_r) +
		    0 +  // 跳过积分项
		    Speed_D_r * (Speed_Erro_r - 2 * Speed_Lasterro_r + Speed_Preverro_r));
	}

	Speed_PID_OUT_r = clamp_pid_output(Speed_PID_OUT_r, PWM_Min, PWM_Max);

	Speed_Preverro_r = Speed_Lasterro_r;
	Speed_Lasterro_r = Speed_Erro_r;

	if (Speed_PID_OUT_r >= 0) {
		Right_Motor_Pwm1(Speed_PID_OUT_r, kRightForwardDir);
	} else {
		Right_Motor_Pwm1(-Speed_PID_OUT_r, !kRightForwardDir);
	}
}

void Motor_Diff_Pid1(void) {
	static float last_turn_error = 0;

	float turn_error = ImageStatus.Det_True - (float)ImageStatus.MiddleLine;
	if (turn_error > -2.0f &&
	    turn_error <
	        2.0f) // 需要注意，这个范围需要根据实际情况调整，过大可能导致小幅度偏差时过度修正，过小可能导致无法修正较大的偏差
	{
		turn_error = 0;
	}

	float current_Kp = Diff_Kp;
	if (turn_error > -10.0f &&
	    turn_error <
	        10.0f) // 同样需要根据实际情况调整这个范围，过大可能导致在较大偏差时过度修正，过小可能导致无法修正较大的偏差
	{
		current_Kp = Diff_Kp * 0.6f;
	}

	float turn_output =
	    current_Kp * turn_error + Diff_Kd * (turn_error - last_turn_error);
	last_turn_error = turn_error;

	if (turn_output > 500.0f) {
		turn_output = 500.0f;
	}
	if (turn_output < -500.0f) {
		turn_output = -500.0f;
	}

	int current_base_speed =
	    Speed_Goal_l - static_cast<int>(my_abs(turn_error) * 3.5f);
	if (current_base_speed < 120) {
		current_base_speed = 120;
	}

	Diff_SpeedL_expect = static_cast<int16_t>(current_base_speed +
	                                          static_cast<int>(turn_output));
	Diff_SpeedR_expect = static_cast<int16_t>(current_base_speed -
	                                          static_cast<int>(turn_output));

	if (Diff_SpeedL_expect < 0) {
		Diff_SpeedL_expect = 0;
	}
	if (Diff_SpeedR_expect < 0) {
		Diff_SpeedR_expect = 0;
	}
	if (Diff_SpeedL_expect > 1500) {
		Diff_SpeedL_expect = 1500;
	}
	if (Diff_SpeedR_expect > 1500) {
		Diff_SpeedR_expect = 1500;
	}
}