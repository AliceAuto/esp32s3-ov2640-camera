# -*- coding: utf-8 -*-
"""OV2640 上位机公共库。

拆分母的动机: live_capture.py(46KB)/udp_capture.py(28KB) 曾是两套各自为政的
单文件客户端, 帧协议解析 / RGB565 转换 / 中继客户端各写一份且已发生漂移
(如 convert_rgb 的舍入公式、OVU 头定义不一致)。本包收敛公共逻辑:
  - frame_parser : FRAME_BEGIN/SIZE/FRAME_END 文本流切帧
  - rgb565       : big-endian RGB565 -> RGB888 (numpy 向量化)
  - (规划) relay_client : 中继信令 + P2P 打洞
  - (规划) netio         : TcpIo/SerialIo 统一传输层
"""
