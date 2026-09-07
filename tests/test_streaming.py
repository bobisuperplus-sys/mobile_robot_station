# -*- coding: utf-8 -*-
"""
GStreamer 双机位视讯推流与网络数据包自动化测试套件
"""

import os
import socket
import time
import unittest
import numpy as np

from simulation.streamer import GStreamerStreamer, MultiCameraStreamServer


class TestGStreamerStreaming(unittest.TestCase):
    """GStreamer 硬件/软件 H.264 RTP 推流管道与网络广播测试"""

    def test_streamer_lifecycle_and_frame_push(self):
        """测试单机位推流器生命周期、帧尺寸校验与管道存活状态"""
        test_port = 15002
        width = 320
        height = 240
        fps = 30

        streamer = GStreamerStreamer(
            width=width,
            height=height,
            fps=fps,
            host="127.0.0.1",
            port=test_port,
            bitrate_kbps=1000,
        )

        try:
            self.assertIsNotNone(streamer.proc, "推流子进程应成功启动")
            self.assertIsNone(streamer.proc.poll(), "推流子进程应处于运行中状态")

            # 1. 写入合法尺寸的伪彩色帧
            valid_frame = np.full((height, width, 3), 128, dtype=np.uint8)
            success = streamer.push_frame(valid_frame)
            self.assertTrue(success, "有效尺寸的帧应成功推入管道")

            # 2. 写入非法尺寸帧，验证防御机制
            invalid_frame = np.full((100, 100, 3), 255, dtype=np.uint8)
            fail_result = streamer.push_frame(invalid_frame)
            self.assertFalse(fail_result, "非法尺寸帧应被安全拦截")

            # 管道应不受非法帧影响继续存活
            self.assertIsNone(streamer.proc.poll(), "管道进程在拦截非法输入后应保持健康存活")

        finally:
            streamer.close()
            self.assertIsNone(streamer.proc, "关闭后进程引用应清空")

    def test_multi_camera_server_udp_reception(self):
        """测试双机位推流调度器在网络层正常广播 RTP H.264 数据包"""
        front_port = 15004
        overview_port = 15006
        width = 320
        height = 240
        fps = 30

        server = MultiCameraStreamServer(
            width=width,
            height=height,
            fps=fps,
            host="127.0.0.1",
            front_port=front_port,
            overview_port=overview_port,
        )

        # 构造本地接收 Socket 监听测试端口
        sock_front = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_front.bind(("127.0.0.1", front_port))
        sock_front.settimeout(2.0)

        sock_overview = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_overview.bind(("127.0.0.1", overview_port))
        sock_overview.settimeout(2.0)

        front_received_bytes = 0
        overview_received_bytes = 0

        try:
            # 持续推送多帧渐变图像以产生稳定的 H.264 编码数据流
            for i in range(15):
                frame_front = np.full((height, width, 3), (i * 15) % 255, dtype=np.uint8)
                frame_overview = np.full((height, width, 3), (255 - i * 15) % 255, dtype=np.uint8)

                server.push_front_frame(frame_front)
                server.push_overview_frame(frame_overview)

                # 接收前向相机包
                try:
                    data_f, _ = sock_front.recvfrom(65535)
                    front_received_bytes += len(data_f)
                    # 验证 RTP 头：首字节高 2 位是版本号 2 (0x80)
                    if len(data_f) >= 12:
                        rtp_version = (data_f[0] >> 6) & 0x03
                        self.assertEqual(rtp_version, 2, "接收到的应为合法 RTP v2 数据报文")
                except socket.timeout:
                    pass

                # 接收全局相机包
                try:
                    data_o, _ = sock_overview.recvfrom(65535)
                    overview_received_bytes += len(data_o)
                    if len(data_o) >= 12:
                        rtp_version = (data_o[0] >> 6) & 0x03
                        self.assertEqual(rtp_version, 2, "接收到的应为合法 RTP v2 数据报文")
                except socket.timeout:
                    pass

                time.sleep(0.01)

            self.assertGreater(front_received_bytes, 0, "前视相机 (front_cam) 应当在网络端口广播出 RTP 视频流")
            self.assertGreater(overview_received_bytes, 0, "全局相机 (overview_cam) 应当在网络端口广播出 RTP 视频流")

        finally:
            sock_front.close()
            sock_overview.close()
            server.close()

    def test_stream_color_fidelity_no_green_screen(self):
        """测试推流端在 rawvideoparse 组帧下的色彩保真度，确保彻底杜绝全绿屏空帧"""
        import subprocess
        import glob
        import cv2

        test_port = 15008
        width = 320
        height = 240
        fps = 30

        # 清理临时文件
        for f in glob.glob("/tmp/test_rx_green_*.jpg"):
            try:
                os.remove(f)
            except OSError:
                pass

        streamer = GStreamerStreamer(
            width=width,
            height=height,
            fps=fps,
            host="127.0.0.1",
            port=test_port,
            bitrate_kbps=1000,
        )

        # 构造黄金接收端截帧管道
        rx_cmd = [
            "gst-launch-1.0", "-q",
            "udpsrc", f"port={test_port}", "buffer-size=2097152",
            'caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96', "!",
            "rtpjitterbuffer", "latency=10", "drop-on-latency=true", "!",
            "rtph264depay", "!",
            "h264parse", "!",
            "avdec_h264", "max-threads=2", "!",
            "videoconvert", "!",
            "jpegenc", "!",
            "multifilesink", "location=/tmp/test_rx_green_%03d.jpg", "max-files=10"
        ]
        rx_proc = subprocess.Popen(rx_cmd)

        # 构造左纯红 (RGB: 255, 0, 0)、右纯蓝 (RGB: 0, 0, 255) 测试帧
        test_frame = np.zeros((height, width, 3), dtype=np.uint8)
        test_frame[:, :width//2] = [255, 0, 0]
        test_frame[:, width//2:] = [0, 0, 255]

        try:
            for _ in range(45):
                streamer.push_frame(test_frame)
                time.sleep(1.0 / fps)
        finally:
            streamer.close()
            time.sleep(0.5)
            rx_proc.terminate()
            try:
                rx_proc.wait(timeout=1.0)
            except:
                rx_proc.kill()

        frames = sorted(glob.glob("/tmp/test_rx_green_*.jpg"))
        self.assertGreater(len(frames), 0, "接收端应成功捕获到视频帧图片")

        # 检查最新接收到的稳定帧
        latest_frame = cv2.imread(frames[-1])
        self.assertIsNotNone(latest_frame, "解码的图像应能被正确读取")

        # 检查像素均值不是绿屏 (B=0, G=135, R=0)
        mean_b = float(latest_frame[:, :, 0].mean())
        mean_g = float(latest_frame[:, :, 1].mean())
        mean_r = float(latest_frame[:, :, 2].mean())

        is_green_screen = (mean_b < 5.0 and mean_r < 5.0 and 120.0 < mean_g < 150.0 and latest_frame.std() < 5.0)
        self.assertFalse(is_green_screen, "接收帧绝不能是未初始化的 YUV 全零绿屏 (B=0, G=135, R=0)")

        # 验证红蓝色彩分割特征
        left_r = float(latest_frame[:, :width//4, 2].mean()) # BGR 的 R 通道
        right_b = float(latest_frame[:, -width//4:, 0].mean()) # BGR 的 B 通道
        self.assertGreater(left_r, 180.0, "左半区域应当保真呈现高饱和红色")
        self.assertGreater(right_b, 180.0, "右半区域应当保真呈现高饱和蓝色")

        # 清理临时文件
        for f in frames:
            try:
                os.remove(f)
            except OSError:
                pass


if __name__ == "__main__":
    unittest.main()
