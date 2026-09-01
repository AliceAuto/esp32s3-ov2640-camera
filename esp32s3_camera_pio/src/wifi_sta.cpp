/*
 * WiFi STA 接入实现(自 main.cpp 拆出)。
 */
#include "wifi_sta.h"
#include "web_cfg.h"    // default_dev_id / handle_web_root / handle_web_save

static uint32_t g_last_ip_log  = 0;
static uint32_t g_sta_last_try = 0;

void setup_wifi() {
  WiFi.setSleep(false);
  log_i("WF entry");

  // 从 NVS 读取上次保存的目标 WiFi；没有记录则用编译期默认
  g_prefs.begin("wifi", true);
  String _ssid = g_prefs.getString("ssid", String(STA_SSID));
  String _pass = g_prefs.getString("pass", String(STA_PASS));
  String _name = g_prefs.getString("name", String(AP_SSID));
  String _id   = g_prefs.getString("id",   default_dev_id());
  String _relay= g_prefs.getString("relay", RELAY_HOST_DEFAULT);   // "host:port", 空=关闭中继
  String _token= g_prefs.getString("token", String(RELAY_TOKEN));  // 中继鉴权口令
  g_prefs.end();
  log_i("WF after nvs");
  _ssid.toCharArray(g_sta_ssid, sizeof(g_sta_ssid));
  _pass.toCharArray(g_sta_pass, sizeof(g_sta_pass));
  _name.toCharArray(g_dev_name, sizeof(g_dev_name));
  _id.toCharArray(g_dev_id,   sizeof(g_dev_id));
  _relay.toCharArray(g_relay_host, sizeof(g_relay_host));
  _token.toCharArray(g_relay_token, sizeof(g_relay_token));
  if (g_relay_host[0] != 0)
    Dbg.printf("[中继] 目标: %s (id=%s token=%s)\n", g_relay_host, g_dev_id,
               g_relay_token[0] ? "已配置" : "无");

  // 凭据加载完再定模式: 无 STA 凭据(NVS 与编译期都没有)时开 SoftAP 兜底,
  // 保证配网页可达、设备不会变成"连不上也配不了"的死机;
  // 有凭据则保持纯 STA —— SoftAP 与 STA 时分复用同一射频, 会明显拉低视频吞吐。
  g_softap_on = (g_sta_ssid[0] == 0);
  if (g_softap_on) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    Dbg.printf("[WiFi] 无 STA 凭据, 已开兜底热点 '%s' 供配网(配好后自动关闭)\n", AP_SSID);
  } else {
    WiFi.mode(WIFI_STA);   // 纯 STA: 彻底关闭 SoftAP, 释放射频给 STA
    Dbg.println("[WiFi] 纯 STA 模式, SoftAP 已彻底关闭");
  }

  cam_server.begin();
  cam_udp.begin(TCP_PORT);
  Dbg.printf("[WiFi] TCP监听: %u\n", TCP_PORT);
  log_i("WF server/udp bound");

  // Web 配置页(通过局域网 IP 访问)
  httpServer.on("/",         HTTP_GET, handle_web_root);
  httpServer.on("/save",     HTTP_GET, handle_web_save);
  httpServer.begin();
  Dbg.println("[WiFi] 配置页已开(局域网 IP:80)");

  if (g_sta_ssid[0] == 0) {
    Dbg.println("[WiFi] 未配置目标 WiFi, 等待通过配置页写入");
    return;
  }
  Dbg.printf("[WiFi] STA 连接 %s ...\n", g_sta_ssid);
  log_i("WF begin %s", g_sta_ssid);
  WiFi.begin(g_sta_ssid, g_sta_pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
    delay(200);
    Dbg.print(".");
  }
  log_i("WF after-connect status=%d", (int)WiFi.status());
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    Dbg.printf("\n[WiFi] STA已连: %s ip=%d.%d.%d.%d\n",
               g_sta_ssid, ip[0], ip[1], ip[2], ip[3]);
    if (g_softap_on) {           // 已入网, 关掉兜底热点把射频还给 STA
      WiFi.softAPdisconnect(true);
      g_softap_on = false;
      Dbg.println("[WiFi] 已入网, 兜底热点关闭");
    }
  } else {
    Dbg.printf("\n[WiFi] STA 连接超时, 设备离线%s\n",
               g_softap_on ? "(兜底热点已开, 可连入配网)" : "(需重连/重新配置)");
  }
}

// 周期打印本机 STA IP（上位机找板子局域网地址用）
void log_wifi_ip() {
  if (WiFi.status() == WL_CONNECTED) {
    uint32_t now = millis();
    if (now - g_last_ip_log >= 5000) {
      g_last_ip_log = now;
      IPAddress ip = WiFi.localIP();
      Dbg.printf("[WiFi] ip=%d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    }
  }
}

// STA 断线自愈：setup_wifi 只在开机时 connect 一次，掉线后无 AP 兜底，
// 这里周期性检测并重连（非阻塞，连接结果由后续状态轮询确认）。
void check_sta_link() {
  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) { g_sta_last_try = now; return; }
  if (g_sta_ssid[0] == 0) return;            // 未配置凭据: 由配网页触发连接
  if (now - g_sta_last_try < 15000) return;
  g_sta_last_try = now;
  Dbg.printf("[WiFi] STA 掉线(status=%d), 尝试重连...\n", WiFi.status());
  WiFi.disconnect();
  delay(50);
  WiFi.begin(g_sta_ssid, g_sta_pass);
}
