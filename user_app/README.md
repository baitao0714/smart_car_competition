# UDP JPEG 上位机

这是一个用于接收龙邱库 UDP 图像数据的上位机程序，支持：

- 纯 JPEG 数据
- 4 字节前缀 + JPEG 数据
- 鼠标移动到图像上时显示当前位置坐标和像素值

## 依赖

```bash
pip install -r requirements.txt
```

你当前可用的解释器是 `D:\Python\python`，并且已经确认包含 `cv2`。

## 运行

默认监听 `0.0.0.0:8080`：

```bash
D:\Python\python udp_jpeg_viewer.py
```

自定义端口：

```bash
D:\Python\python udp_jpeg_viewer.py --port 8080
```

自定义显示窗口最大尺寸：

```bash
D:\Python\python udp_jpeg_viewer.py --port 8080 --max-width 1280 --max-height 720
```

## 发送端要求

发送端每个 UDP 数据包应包含一整帧 JPEG 数据。程序会自动寻找 JPEG 起始标记 `FF D8`，因此可以兼容：

- 纯 JPEG
- 4 字节长度头 + JPEG
- 4 字节其他前缀 + JPEG

如果发送端把一帧拆成多个 UDP 包，这个程序无法自动重组，需要先把发送端改成单包一帧。

## 鼠标显示说明

- 鼠标移动到图像上，界面左上角会显示：
  - 当前坐标
  - BGR 像素值
- 如果窗口做了缩放，程序会自动把鼠标位置映射回原图坐标。
