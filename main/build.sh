#!/bin/bash

##############################################################################
# 脚本名称：build.sh
# 脚本功能：自动查找依赖库路径 --> 配置PKG_CONFIG_PATH --> 编译项目
# 适用场景：龙芯2K300/301平台，依赖 ncnn 和 OpenCV 的项目编译
# 使用说明：
#       1. 仅编译：./build.sh
#       2. 编译 + 传输到开发板：./build.sh 192.168.1.100
#       3. 编译 + 传输到开发板 + 运行：./build.sh 192.168.1.100 -r
##############################################################################

# ====================================================================================================================================================== #
# ============================================================== 基础配置（绝对路径）=================================================================== #
# ====================================================================================================================================================== #
# 交叉编译工具链配置（绝对路径）
TOOLCHAIN_ABS_PATH="/home/wuwu/workspace/tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
TOOLCHAIN_CMAKE_MACRO_FILE="./toolchain_path.cmake"

# 依赖库配置（绝对路径）
OPENCV_ABS_PATH="/home/wuwu/workspace/tools/LQ_Dep_libs/opencv_install"
NCNN_ABS_PATH="/home/wuwu/workspace/tools/LQ_Dep_libs/ncnn_install"
FFMPEG_ABS_PATH="/home/wuwu/workspace/tools/LQ_Dep_libs/ffmpeg_install"

# 依赖库配置（用于 pkgconfig）
DEP_LIBS="opencv_install ncnn_install ffmpeg_install"
PKG_REL_PATH="lib/pkgconfig/"

# 路径配置（项目在共享目录）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLS_DIR="${SCRIPT_DIR}/tools"
TARGET_DIR="LQ_Dep_libs"

# 编译参数配置
BUILD_THREADS=$(nproc)
BUILD_DIR="build"

# SCP 传输配置
BOARD_USER="root"
BOARD_TARGET_PATH="/home/root"
EXECUTABLE_NAME="main"
REMOTE_EXEC_CMD="${BOARD_TARGET_PATH}/${EXECUTABLE_NAME}"
BOARD_PHYSICAL_TTY="/dev/console"
BOARD_LOG_FILE="/tmp/${EXECUTABLE_NAME}.log"

# ====================================================================================================================================================== #
# ===================================================================== 核心函数定义 ===================================================================== #
# ====================================================================================================================================================== #

function log_info() {
    echo -e "\033[32m[$(date +%Y-%m-%d\ %H:%M:%S)] [INFO ] $1\033[0m"
}

function log_warn() {
    echo -e "\033[33m[$(date +%Y-%m-%d\ %H:%M:%S)] [WARN ] $1\033[0m"
}

function log_error() {
    echo -e "\033[31m[$(date +%Y-%m-%d\ %H:%M:%S)] [ERROR] $1\033[0m"
    exit 1
}

function check_and_install_deps() {
    local missing_deps=()
    local dep_list=("$@")
    
    log_info "=================================================================== 检测依赖工具 ==================================================================="
    for dep in "${dep_list[@]}"; do
        if command -v "${dep}" &>/dev/null; then
            log_info "✅ 已安装 ${dep}"
        else
            log_warn "⚠️ 未安装 ${dep}！"
            missing_deps+=("${dep}")
        fi
    done
    
    if [[ ${#missing_deps[@]} -eq 0 ]]; then
        log_info "✅ 所有依赖都已安装"
        return 0
    fi
    
    log_info "🔧 安装缺失依赖..."
    sudo apt update -y
    for dep in "${missing_deps[@]}"; do
        sudo apt install -y "${dep}" || log_error "❌ 安装 ${dep} 失败"
    done
    log_info "✅ 依赖安装完成"
}

# 验证依赖库路径
function verify_lib_paths() {
    log_info "=================================================================== 验证依赖库路径 ==================================================================="
    
    if [ -d "${OPENCV_ABS_PATH}" ]; then
        log_info "✅ OpenCV 路径: ${OPENCV_ABS_PATH}"
        if [ -d "${OPENCV_ABS_PATH}/lib/pkgconfig" ]; then
            log_info "   pkgconfig 目录存在"
        else
            log_warn "⚠️ OpenCV pkgconfig 目录不存在"
        fi
    else
        log_error "❌ OpenCV 目录不存在: ${OPENCV_ABS_PATH}"
    fi
    
    if [ -d "${NCNN_ABS_PATH}" ]; then
        log_info "✅ ncnn 路径: ${NCNN_ABS_PATH}"
    else
        log_error "❌ ncnn 目录不存在: ${NCNN_ABS_PATH}"
    fi
    
    if [ -d "${FFMPEG_ABS_PATH}" ]; then
        log_info "✅ ffmpeg 路径: ${FFMPEG_ABS_PATH}"
    else
        log_warn "⚠️ ffmpeg 目录不存在（非致命）"
    fi
    
    log_info "======================================================================================================================================================"
}

function setup_pkgconfig_path() {
    log_info "================================================================ 配置 PKG_CONFIG_PATH ================================================================"
    
    # 直接使用绝对路径构建 PKG_CONFIG_PATH
    local pkg_path_list=""
    
    # OpenCV pkgconfig
    if [ -d "${OPENCV_ABS_PATH}/${PKG_REL_PATH}" ]; then
        pkg_path_list+="${OPENCV_ABS_PATH}/${PKG_REL_PATH}:"
        log_info "✅ 添加 OpenCV pkgconfig: ${OPENCV_ABS_PATH}/${PKG_REL_PATH}"
    else
        log_warn "⚠️ OpenCV pkgconfig 目录不存在"
    fi
    
    # ncnn pkgconfig
    if [ -d "${NCNN_ABS_PATH}/${PKG_REL_PATH}" ]; then
        pkg_path_list+="${NCNN_ABS_PATH}/${PKG_REL_PATH}:"
        log_info "✅ 添加 ncnn pkgconfig: ${NCNN_ABS_PATH}/${PKG_REL_PATH}"
    else
        log_warn "⚠️ ncnn pkgconfig 目录不存在"
    fi
    
    # ffmpeg pkgconfig
    if [ -d "${FFMPEG_ABS_PATH}/${PKG_REL_PATH}" ]; then
        pkg_path_list+="${FFMPEG_ABS_PATH}/${PKG_REL_PATH}:"
        log_info "✅ 添加 ffmpeg pkgconfig: ${FFMPEG_ABS_PATH}/${PKG_REL_PATH}"
    else
        log_warn "⚠️ ffmpeg pkgconfig 目录不存在"
    fi
    
    # 导出 PKG_CONFIG_PATH
    export PKG_CONFIG_PATH="${pkg_path_list}${PKG_CONFIG_PATH}"
    export PKG_CONFIG_PATH=$(echo "${PKG_CONFIG_PATH}" | sed 's/:$//')
    
    log_info "✅ PKG_CONFIG_PATH: ${PKG_CONFIG_PATH}"
    log_info "======================================================================================================================================================"
    
    # 验证 pkg-config
    log_info "=============================================================== 验证 pkg-config 可用性 ==============================================================="
    for lib in opencv4 ncnn libavformat libavcodec; do
        if pkg-config --exists "${lib}" 2>/dev/null; then
            log_info "✅ pkg-config 验证成功: ${lib}"
            log_info "  ├─ 编译参数: $(pkg-config --cflags ${lib})"
            log_info "  └─ 链接参数: $(pkg-config --libs ${lib})"
        else
            log_warn "⚠️ pkg-config 未找到 ${lib} 库（非致命）"
        fi
    done
    log_info "======================================================================================================================================================"
}

function is_valid_ip() {
    local ip=$1
    local ip_regex="^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$"
    [[ $ip =~ $ip_regex ]] && return 0 || return 1
}

function scp_to_board() {
    local board_ip=$1
    local local_exec_path="${BUILD_DIR}/${EXECUTABLE_NAME}"
    
    if [ ! -f "${local_exec_path}" ]; then
        log_error "❌ 可执行程序不存在: ${local_exec_path}"
    fi
    
    log_info "🔧 测试与开发板 ${board_ip} 的连通性..."
    ping -c 2 -W 3 "${board_ip}" >/dev/null 2>&1 || log_warn "⚠️ 开发板无法 ping 通"
    
    log_info "🔧 传输可执行程序到 ${board_ip}:${BOARD_TARGET_PATH}"
    if scp -O -o ConnectTimeout=10 "${local_exec_path}" "${BOARD_USER}@${board_ip}:${BOARD_TARGET_PATH}"; then
        log_info "✅ 传输成功"
        ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "chmod +x ${REMOTE_EXEC_CMD}" >/dev/null 2>&1
    else
        log_error "❌ 传输失败"
    fi
}

function stop_remote_program() {
    local board_ip=$1
    log_info "================================================================== 停止开发板旧程序 =================================================================="
    local pid=$(ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "pgrep -f ${EXECUTABLE_NAME}" 2>/dev/null)
    if [ -n "${pid}" ]; then
        log_info "停止程序 PID: ${pid}"
        ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "kill -9 ${pid}" >/dev/null 2>&1
    else
        log_warn "未检测到运行中的程序"
    fi
}

function run_remote_program() {
    local board_ip=$1
    log_info "==================== 远程执行程序 ===================="
    log_info "🔧 在开发板 ${board_ip} 执行程序: ${REMOTE_EXEC_CMD}"
    
    local temp_script="${BOARD_TARGET_PATH}/run_app.sh"
    local remote_script_content="
        #!/bin/bash
        echo '===== 开始执行程序 =====' >> ${BOARD_LOG_FILE}
        pkill -f ${EXECUTABLE_NAME} >/dev/null 2>&1 || true
        if [ ! -f ${REMOTE_EXEC_CMD} ]; then
            echo '错误：程序不存在！' >> ${BOARD_LOG_FILE}
            exit 1
        fi
        chmod +x ${REMOTE_EXEC_CMD}
        stdbuf -o0 -e0 nohup ${REMOTE_EXEC_CMD} > ${BOARD_PHYSICAL_TTY} 2>&1 >> ${BOARD_LOG_FILE} &
        sleep 1
        PID=\$(pgrep -f '${EXECUTABLE_NAME}')
        if [ -n \"\$PID\" ]; then
            echo \"程序启动成功！PID：\$PID\" >> ${BOARD_LOG_FILE}
            exit 0
        else
            echo '程序启动失败！' >> ${BOARD_LOG_FILE}
            exit 1
        fi
    "
    
    echo "${remote_script_content}" | ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "cat > ${temp_script} && chmod +x ${temp_script}"
    ssh -o ConnectTimeout=20 "${BOARD_USER}@${board_ip}" "bash ${temp_script}" || log_error "❌ 程序启动失败"
    ssh "${BOARD_USER}@${board_ip}" "rm -f ${temp_script}" >/dev/null 2>&1
    log_info "✅ 程序启动成功"
}

# ====================================================================================================================================================== #
# ==================================================================== 主程序 ======================================================================== #
# ====================================================================================================================================================== #

# 解析传入参数
BOARD_IP=$1
RUN_FLAG=$2
RUN_PROGRAM=false

if [ -n "${BOARD_IP}" ]; then
    if ! is_valid_ip "${BOARD_IP}"; then
        log_error "❌ 无效的 IP 地址: ${BOARD_IP}"
    fi
    if [ "${RUN_FLAG}" = "-r" ]; then
        RUN_PROGRAM=true
        log_info "🔧 本次执行：编译 → 传输 → 远程运行程序"
    else
        log_info "🔧 本次执行：编译 → 传输（不运行）"
    fi
else
    log_info "🔧 本次执行：仅编译"
fi

set -e

# 打印配置信息
log_info "================================================================= 配置信息 ================================================================="
log_info "项目根目录: ${SCRIPT_DIR}"
log_info "工具链路径: ${TOOLCHAIN_ABS_PATH}"
log_info "OpenCV 路径: ${OPENCV_ABS_PATH}"
log_info "ncnn 路径: ${NCNN_ABS_PATH}"
log_info "ffmpeg 路径: ${FFMPEG_ABS_PATH}"
log_info "======================================================================================================================================================"

# 检查依赖
function main() {
    local REQUIRED_DEPS=("pkg-config" "cmake")
    check_and_install_deps "${REQUIRED_DEPS[@]}"
}
main

# 验证路径
verify_lib_paths

# 检查工具链
log_info "================================================================= 检查交叉编译工具链 ================================================================="
if [ -d "${TOOLCHAIN_ABS_PATH}" ]; then
    log_info "✅ 工具链路径: ${TOOLCHAIN_ABS_PATH}"
    if [ -f "${TOOLCHAIN_ABS_PATH}/bin/loongarch64-linux-gnu-gcc" ]; then
        log_info "✅ gcc 存在: ${TOOLCHAIN_ABS_PATH}/bin/loongarch64-linux-gnu-gcc"
    else
        log_error "❌ 工具链中未找到 gcc"
    fi
else
    log_error "❌ 工具链目录不存在: ${TOOLCHAIN_ABS_PATH}"
fi
log_info "======================================================================================================================================================"

# 生成 CMake 宏文件
log_info "================================================================== CMake 宏文件生成 =================================================================="
cat > "${TOOLCHAIN_CMAKE_MACRO_FILE}" << EOF
set(CMAKE_TOOLCHAIN_PATH "${TOOLCHAIN_ABS_PATH}" CACHE PATH "Loongson toolchain path" FORCE)
set(OpenCV_DIR "${OPENCV_ABS_PATH}/lib/cmake/opencv4" CACHE PATH "OpenCV cmake path" FORCE)
set(ncnn_DIR "${NCNN_ABS_PATH}/lib/cmake/ncnn" CACHE PATH "ncnn cmake path" FORCE)
EOF
log_info "✅ CMake 宏文件生成: ${TOOLCHAIN_CMAKE_MACRO_FILE}"
cat "${TOOLCHAIN_CMAKE_MACRO_FILE}"
log_info "======================================================================================================================================================"

# 配置 PKG_CONFIG_PATH
setup_pkgconfig_path

# 将工具链 bin 目录添加到 PATH（确保使用交叉编译器的 as/ld 等工具）
export PATH="${TOOLCHAIN_ABS_PATH}/bin:${PATH}"
log_info "✅ 工具链已添加到 PATH"

# 编译项目
log_info "====================================================================== 编译项目 ======================================================================"
if [ -d "${BUILD_DIR}" ]; then
    log_info "删除旧的 build 目录"
    rm -rf "${BUILD_DIR}"
fi

log_info "创建构建目录并配置 cmake"
cmake -B "${BUILD_DIR}" \
    -DLQ_ENABLE_OPENCV=ON \
    -DLQ_OPENCV_ROOT="${OPENCV_ABS_PATH}" \
    -DLQ_ENABLE_NCNN=ON \
    -DLQ_NCNN_ROOT="${NCNN_ABS_PATH}" \
    || log_error "❌ cmake 配置失败"

log_info "开始编译（线程数: ${BUILD_THREADS}）"
cmake --build "${BUILD_DIR}" -j"${BUILD_THREADS}" || log_error "❌ 编译失败"
log_info "✅ 编译完成"
log_info "======================================================================================================================================================"

# 传输到开发板
if [ -n "${BOARD_IP}" ]; then
    stop_remote_program "${BOARD_IP}"
    scp_to_board "${BOARD_IP}"
    if [ "${RUN_PROGRAM}" = true ]; then
        run_remote_program "${BOARD_IP}"
    fi
fi

log_info "🎉 脚本执行完成！"
log_info "🔍 可执行程序: ${SCRIPT_DIR}/${BUILD_DIR}/${EXECUTABLE_NAME}"
exit 0