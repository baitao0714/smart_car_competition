#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UDP JPEG 图像接收与查看器.

功能:
- 监听 UDP 端口接收图像数据
- 自动识别以下常见格式:
  1) 纯 JPEG 数据
  2) 4 字节头 + JPEG 数据(如长度头/宽高头)
- 显示图像
- 鼠标移动到图像上时，显示当前位置坐标和像素值

依赖:
- Python 3.8+
- opencv-python
- numpy
"""

from __future__ import annotations

import argparse
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


WINDOW_NAME = "UDP JPEG Viewer"
MAX_HEADER_SCAN = 16
DEFAULT_BIND_IP = "192.168.193.172"
DEFAULT_PORT = 8080
DEFAULT_MAX_DISPLAY_WIDTH = 1600
DEFAULT_MAX_DISPLAY_HEIGHT = 1200


@dataclass
class ViewerState:
    frame: Optional[np.ndarray] = None
    frame_size: Tuple[int, int] = (0, 0)
    frame_count: int = 0
    decode_count: int = 0
    last_packet_size: int = 0
    last_update_ts: float = 0.0
    mouse_x: int = -1
    mouse_y: int = -1
    mouse_bgr: Tuple[int, int, int] = (-1, -1, -1)
    mouse_info: str = "等待图像..."
    display_ratio: float = 1.0
    stop: bool = False


state = ViewerState()
state_lock = threading.Lock()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UDP JPEG image viewer with mouse pixel probe")
    parser.add_argument("--bind-ip", default=DEFAULT_BIND_IP, help="绑定地址，默认 0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="监听端口，默认 8080")
    parser.add_argument("--max-width", type=int, default=DEFAULT_MAX_DISPLAY_WIDTH, help="最大显示宽度")
    parser.add_argument("--max-height", type=int, default=DEFAULT_MAX_DISPLAY_HEIGHT, help="最大显示高度")
    return parser.parse_args()


def find_jpeg_payload(data: bytes) -> bytes:
    """从 UDP 数据中提取 JPEG 载荷。

    兼容:
    - 纯 JPEG: 直接以 FF D8 开头
    - 4 字节头 + JPEG: JPEG 从第 5 个字节开始
    - 其他前置头: 在前 16 字节内搜索 FF D8
    """
    if not data:
        return data

    if data.startswith(b"\xff\xd8"):
        return data

    if len(data) >= 6 and data[4:6] == b"\xff\xd8":
        return data[4:]

    scan_limit = min(len(data), MAX_HEADER_SCAN)
    soi_index = data.find(b"\xff\xd8", 0, scan_limit)
    if soi_index >= 0:
        return data[soi_index:]

    return data


def decode_jpeg(data: bytes) -> Optional[np.ndarray]:
    payload = find_jpeg_payload(data)
    if len(payload) < 4:
        return None

    buffer = np.frombuffer(payload, dtype=np.uint8)
    frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
    return frame


def receiver_thread(bind_ip: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, port))
    sock.settimeout(0.5)

    print(f"[INFO] Listening on {bind_ip}:{port}")

    while True:
        with state_lock:
            if state.stop:
                break

        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break

        frame = decode_jpeg(data)
        if frame is None:
            with state_lock:
                state.last_packet_size = len(data)
                state.decode_count += 1
                state.mouse_info = f"接收到 {len(data)} 字节，但 JPEG 解码失败"
            continue

        h, w = frame.shape[:2]
        now = time.time()
        with state_lock:
            state.frame = frame
            state.frame_size = (w, h)
            state.frame_count += 1
            state.decode_count += 1
            state.last_packet_size = len(data)
            state.last_update_ts = now
            if state.mouse_x >= 0 and state.mouse_y >= 0:
                update_mouse_info_locked()
            else:
                state.mouse_info = f"已接收图像 {w}x{h}，等待鼠标移动..."

    sock.close()


def update_mouse_info_locked() -> None:
    frame = state.frame
    if frame is None:
        state.mouse_info = "等待图像..."
        return

    fx, fy = state.frame_size
    ratio = state.display_ratio if state.display_ratio > 0 else 1.0
    x = int(state.mouse_x / ratio)
    y = int(state.mouse_y / ratio)

    if x < 0 or y < 0 or x >= fx or y >= fy:
        state.mouse_info = f"坐标({x}, {y}) 超出图像范围"
        state.mouse_bgr = (-1, -1, -1)
        return

    b, g, r = frame[y, x].tolist()
    state.mouse_bgr = (int(b), int(g), int(r))
    state.mouse_info = f"坐标: ({x}, {y})  BGR: ({b}, {g}, {r})"


def on_mouse(event: int, x: int, y: int, flags: int, param) -> None:
    del flags, param
    if event != cv2.EVENT_MOUSEMOVE:
        return

    with state_lock:
        state.mouse_x = x
        state.mouse_y = y
        update_mouse_info_locked()


def resize_for_display(frame: np.ndarray, max_width: int, max_height: int) -> Tuple[np.ndarray, float]:
    h, w = frame.shape[:2]
    ratio = min(max_width / float(w), max_height / float(h), 1.0)
    if ratio >= 1.0:
        return frame, 1.0

    new_w = max(1, int(w * ratio))
    new_h = max(1, int(h * ratio))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return resized, ratio


def draw_overlay(frame: np.ndarray, info: str, packet_size: int, decode_count: int, fps_text: str) -> np.ndarray:
    canvas = frame.copy()
    overlay_lines = [info]

    x0, y0 = 6, 15
    line_h = 10
    box_w = max(150, min(200, canvas.shape[1] - 16))
    box_h = 6 + line_h * len(overlay_lines)

    cv2.rectangle(canvas, (4, 4), (4 + box_w, 4 + box_h), (0, 0, 0), thickness=-1)
    cv2.rectangle(canvas, (4, 4), (4 + box_w, 4 + box_h), (0, 255, 255), thickness=1)

    for idx, line in enumerate(overlay_lines):
        y = y0 + idx * line_h
        cv2.putText(canvas, line, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1, cv2.LINE_AA)

    return canvas


def main() -> int:
    args = parse_args()

    recv_thread = threading.Thread(target=receiver_thread, args=(args.bind_ip, args.port), daemon=True)
    recv_thread.start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    last_frame_count = 0
    last_fps_ts = time.time()
    fps_value = 0.0

    try:
        while True:
            with state_lock:
                if state.stop:
                    break
                frame = None if state.frame is None else state.frame.copy()
                frame_size = state.frame_size
                packet_size = state.last_packet_size
                decode_count = state.decode_count
                mouse_info = state.mouse_info

            if frame is None:
                blank = np.zeros((240, 360, 3), dtype=np.uint8)
                cv2.putText(blank, f"Waiting for UDP data on {args.bind_ip}:{args.port}", (12, 54),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(blank, "Press Q or ESC to quit.", (12, 86),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(WINDOW_NAME, blank)
            else:
                display_frame, ratio = resize_for_display(frame, args.max_width, args.max_height)
                with state_lock:
                    state.display_ratio = ratio
                    if state.mouse_x >= 0 and state.mouse_y >= 0:
                        update_mouse_info_locked()
                    mouse_info = state.mouse_info

                now = time.time()
                elapsed = now - last_fps_ts
                if elapsed >= 1.0:
                    current_frame_count = decode_count
                    fps_value = (current_frame_count - last_frame_count) / elapsed
                    last_frame_count = current_frame_count
                    last_fps_ts = now

                fps_text = f"解码速率: {fps_value:.2f} fps"
                annotated = draw_overlay(
                    display_frame,
                    mouse_info,
                    packet_size,
                    decode_count,
                    fps_text,
                )
                title = f"UDP JPEG Viewer - frame={frame_size[0]}x{frame_size[1]} scale={ratio:.3f}"
                cv2.setWindowTitle(WINDOW_NAME, title)
                cv2.imshow(WINDOW_NAME, annotated)

            key = cv2.waitKey(15) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                break
    finally:
        with state_lock:
            state.stop = True
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
