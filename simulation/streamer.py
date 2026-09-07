#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GStreamer 工业级超低延迟视讯推流模块 (GStreamer RTP/UDP Streamer)
基于 Linux 原生 GStreamer 硬件/软件 H.264 编码管道，将仿真车载相机画面低延迟分发至上位机。
遵循 gstreamer-streaming-expert 黄金管道规范：tune=zerolatency, bframes=0, 延迟控制在 30-60ms。
"""

import sys
import subprocess
import atexit
import numpy as np


class GStreamerStreamer:
    """
    单机位 GStreamer 视讯流推流器
    特性:
      - 纯原生 gst-launch 管道子进程，零依赖外部特定编译版 OpenCV
      - 显式 Caps 协商: 输入强制对齐 MuJoCo 原生 RGB 内存排布，杜绝红蓝色彩倒置
      - 极低延迟保证: 关闭 B 帧、关闭平滑缓存、强制 IDR 关键帧周期为 15 帧
      - 具备生命周期防护: 程序退出时自动关闭输入流与子进程，释放 UDP 端口
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        host: str = "127.0.0.1",
        port: int = 5002,
        bitrate_kbps: int = 2000,
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.host = str(host)
        self.port = int(port)
        self.bitrate_kbps = int(bitrate_kbps)
        self.expected_bytes = self.width * self.height * 3

        self.proc = None
        self._start_pipeline()
        atexit.register(self.close)

    def _start_pipeline(self):
        """构建并拉起黄金推流管道"""
        pipeline_cmd = [
            "gst-launch-1.0",
            "-q",
            "fdsrc",
            "fd=0",
            "do-timestamp=true",
            "!",
            "rawvideoparse",
            "format=rgb",
            f"width={self.width}",
            f"height={self.height}",
            f"framerate={self.fps}/1",
            "!",
            "videoconvert",
            "!",
            "video/x-raw,format=I420",
            "!",
            "x264enc",
            "tune=zerolatency",
            "speed-preset=ultrafast",
            f"bitrate={self.bitrate_kbps}",
            "key-int-max=15",
            "bframes=0",
            "sliced-threads=true",
            "!",
            "h264parse",
            "!",
            "rtph264pay",
            "config-interval=1",
            "pt=96",
            "!",
            "udpsink",
            f"host={self.host}",
            f"port={self.port}",
            "sync=false",
            "async=false",
        ]

        try:
            self.proc = subprocess.Popen(
                pipeline_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except FileNotFoundError as err:
            raise RuntimeError(
                "未在系统中找到 gst-launch-1.0 命令，请先执行:\n"
                "sudo apt update && sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly"
            ) from err

    def push_frame(self, rgb_frame: np.ndarray) -> bool:
        """
        向 GStreamer 管道写入单帧 RGB 像素流
        :param rgb_frame: 形状为 (height, width, 3)，数据类型为 uint8 的 NumPy 数组
        :return: 写入成功返回 True，失败返回 False
        """
        if self.proc is None or self.proc.stdin is None:
            return False

        if self.proc.poll() is not None:
            # 管道进程已异常退出
            return False

        try:
            raw_bytes = rgb_frame.tobytes()
            if len(raw_bytes) != self.expected_bytes:
                return False
            self.proc.stdin.write(raw_bytes)
            self.proc.stdin.flush()
            return True
        except (BrokenPipeError, IOError):
            return False

    def close(self):
        """优雅关闭推流管道子进程"""
        if self.proc is not None:
            try:
                if self.proc.stdin:
                    self.proc.stdin.close()
                if self.proc.stderr:
                    self.proc.stderr.close()
                self.proc.terminate()
                self.proc.wait(timeout=1.0)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            finally:
                self.proc = None


class MultiCameraStreamServer:
    """
    多机位视频流推流调度管理器
    默认管理两路关键机位:
      - front_cam: 车载前视第一人称驾驶感知视角 (UDP: 5002)
      - overview_cam: 全局高空透视监控视角 (UDP: 5004)
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        host: str = "127.0.0.1",
        front_port: int = 5002,
        overview_port: int = 5004,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.host = host

        self.streamer_front = GStreamerStreamer(
            width=width, height=height, fps=fps, host=host, port=front_port
        )
        self.streamer_overview = GStreamerStreamer(
            width=width, height=height, fps=fps, host=host, port=overview_port
        )

    def push_front_frame(self, rgb_frame: np.ndarray) -> bool:
        """推送前视感知画面"""
        return self.streamer_front.push_frame(rgb_frame)

    def push_overview_frame(self, rgb_frame: np.ndarray) -> bool:
        """推送全局监控画面"""
        return self.streamer_overview.push_frame(rgb_frame)

    def close(self):
        """关闭所有机位推流管道"""
        if self.streamer_front:
            self.streamer_front.close()
        if self.streamer_overview:
            self.streamer_overview.close()
