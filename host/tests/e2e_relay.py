# -*- coding: utf-8 -*-
"""端到端验证: 用真实 relay_client + core 连模拟中继, 收帧 + 下发切换命令。"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lib import core
from lib import relay_client

core.RELAY_SPEC = "127.0.0.1:8001"
core.DEV_ID = "cam-test"
core.G["res_code"] = "t"
core.G["auto_q"] = True

# 连接中继(走真实 RelayIo 握手)
relay = relay_client.open_relay(core.RELAY_SPEC, core.DEV_ID, core.G["stop"])
if relay is None:
    print("[FAIL] 中继连接失败")
    sys.exit(1)

core.G["relay"] = relay
core._relay_on[0] = True
print("[OK] 中继握手成功, cam_id =", relay.cam_id)

# 验证状态徽章判定逻辑
connected = core.G["tcp"] is not None or core.G["relay"] is not None
print("[OK] connected =", connected, "(应为 True)")

# 收帧: 用真实 frame_parser 走 core.pump 路径
frames = 0
t0 = time.time()
while time.time() - t0 < 3.0:
    data = relay.read(65536)
    if data:
        core._pump_bytes(data)
        frames = core.G["frames"]

print(f"[OK] 3 秒内收到完整帧: {frames} (应 > 0), last_good 刷新:", core.G["last_good"] > 0)

# 验证切换命令格式: 中继模式应发 vt (无 U/N 后缀)
# 直接调用 reader 侧同款逻辑: _resync_bytes
cmd = core._resync_bytes()
print("[OK] 中继切换命令:", cmd, "(应为 b'vt')")
core.send_cmd(cmd)
time.sleep(0.5)

relay.close()
core.G["relay"] = None
print("[DONE]")
