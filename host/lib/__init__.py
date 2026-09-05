# -*- coding: utf-8 -*-
"""OV2640 上位机公共库。

拆分母的动机: 曾经的 live_capture.py(46KB)/udp_capture.py(28KB) 是两套各自为政的
单文件客户端, 帧协议解析 / RGB565 转换 / 中继客户端各写一份且已发生漂移
(如 convert_rgb 的舍入公式、OVU 头定义不一致)。现收敛到 host/lib 并由
main.py(主预览) / tools/preview_serial.py(串口轮询) 共用:
  - core          : 运行时配置 + 共享状态 + 帧路由 + 命令 + 统计
  - reader        : 收流编排(局域网 UDP/TCP、跨网中继、自动回退)
  - gui           : Tkinter 界面 + 后台解码 + 渲染循环 + 拍照/存图
  - frame_parser  : FRAME_BEGIN/SIZE/FRAME_END 文本流切帧
  - protocol      : OVU UDP 分片协议 + 分辨率表 + 分片重组器
  - rgb565        : big-endian RGB565 -> RGB888 (numpy 向量化)
  - relay_client  : 中继服务端观众端(RelayIo, 4B 长度前缀消息流解包)
  - netio         : 局域网发现 + TCP 命令/流通道连接
  - watchdog      : 色彩异常(蓝紫偏)看门狗
  - gui_style     : 上位机共用的深色现代化主题(配色/字体/控件样式)
"""
