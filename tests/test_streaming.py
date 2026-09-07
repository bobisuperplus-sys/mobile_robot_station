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


if __name__ == "__main__":
    unittest.main()
