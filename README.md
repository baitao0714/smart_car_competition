# 龙芯 2K0300 / 2K301 基础库与视觉小车工程

面向龙芯 **2K0300 / 2K301** 平台的嵌入式开发库与智能车（视觉寻迹 + 元素识别）运行工程，基于 C/C++ 开发，使用交叉编译工具链构建，可运行于 Linux 目标板。

---

## 目录结构

```
Loongson_2K300_301_LIB_new/
├── main/                    # 主工程（CMake 构建入口 + 主程序）
│   ├── main.cpp             # 程序入口，通过宏切换正式运行 / 测试例程
│   ├── main.hpp             # 汇总包含所有库头文件
│   ├── CMakeLists.txt       # CMake 构建脚本
│   ├── build.sh             # 一键编译 / 上传 / 运行脚本
│   ├── toolchain_path.cmake # 交叉编译工具链配置
│   └── 更新日志.md          # 变更日志
├── libraries/
│   ├── app/                 # 应用层（整车运行、图像、电机、无刷、显示、陀螺仪等）
│   ├── common/              # 通用定义与工具
│   └── drv/                 # 底层驱动（GPIO/PWM/ADC/CAN/UART/SPI/I2C/Timer/NCNN...）
├── example/                 # 测试例程（23 个 demo）
├── driver/                  # 内核态设备驱动（I2C 传感器、LCD 等）+ 上传脚本
├── user_app/                # 用户个人程序目录
├── docs/                    # 文档
├── element_roi.conf         # 元素识别 ROI 配置
├── line_crop.conf           # 图像裁剪配置
└── pid.conf                 # PID 参数配置（运行时可热加载）
```

---

## 环境依赖

- **交叉编译工具链**：`loongarch64-linux-gnu`（例如 `loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6`）
- **CMake** ≥ 3.16
- **可选依赖**：
  - OpenCV（用于图像处理，开关 `LQ_ENABLE_OPENCV`）
  - NCNN（用于神经网络推理，开关 `LQ_ENABLE_NCNN`，需在启用 OpenCV 的前提下）
  - FFmpeg（图像传输相关）

---

## 快速开始

### 1. 配置工具链与依赖库路径

编辑 `main/build.sh` 顶部的绝对路径配置：

```bash
TOOLCHAIN_ABS_PATH="/path/to/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
OPENCV_ABS_PATH="/path/to/LQ_Dep_libs/opencv_install"
NCNN_ABS_PATH="/path/to/LQ_Dep_libs/ncnn_install"
FFMPEG_ABS_PATH="/path/to/LQ_Dep_libs/ffmpeg_install"
```

### 2. 编译

```bash
cd main

# 仅编译（增量）
./build.sh

# 清理后重新编译
./build.sh -c
```

### 3. 上传并运行到开发板

开发板与 PC 需处于同一网络：

```bash
# 编译 + 传输到开发板
./build.sh 192.168.1.100

# 编译 + 传输 + 运行（后台运行，日志见板卡 /tmp/ 目录）
./build.sh 192.168.1.100 -r

# 清理重建 + 传输 + 运行
./build.sh -c 192.168.1.100 -r
```

构建产物为 `main/build/main`。

---

## 切换运行目标

在 `main/main.cpp` 中通过宏切换正式运行或测试例程：

| 宏 | 说明 |
| --- | --- |
| `LQ_APP_TARGET` | 正式运行目标（默认 `LQ_APP_CAR_RUNTIME`） |
| `LQ_DEMO_TARGET` | 测试例程目标（默认 `LQ_DEMO_NONE`） |

示例：切换为电机测试例程：

```cpp
#define LQ_DEMO_TARGET LQ_DEMO_MOTOR
```

### 可选测试例程

`example/src/` 下提供 23 个 demo：GPIO、PWM、定时器 PWM、编码器、CANFD、NCNN、IPS 屏、MPU6050、LSM6DSR、VL53L0X、UDP 图传 / 波形、ICM42688、NTP、定时器、模块加载、电机、图像、无刷等。

---

## 可选功能开关（CMake）

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| `LQ_ENABLE_OPENCV` | `OFF` | 启用 OpenCV 相关模块 |
| `LQ_ENABLE_NCNN` | `OFF` | 启用 NCNN 相关模块（依赖 OpenCV） |

---

## 配置文件

| 文件 | 说明 |
| --- | --- |
| `pid.conf` | PID 参数，运行时可热加载（缺少时自动创建） |
| `element_roi.conf` | 元素识别 ROI 配置 |
| `line_crop.conf` | 图像裁剪配置 |

---

## 上位机

`user_app/` 下提供 UDP JPEG 图传上位机，详见 `user_app/README.md`：

```bash
pip install -r user_app/requirements.txt
python user_app/udp_jpeg_viewer.py
```

---

## 更新日志

详细变更记录见 `main/更新日志.md`。
