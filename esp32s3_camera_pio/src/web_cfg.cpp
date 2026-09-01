/*
 * Web 配置页实现(自 main.cpp 拆出)。
 */
#include "web_cfg.h"

// ---- Web 配置页 + 目标 WiFi(NVS) ----
char        g_sta_ssid[33] = STA_SSID;   // 运行时 STA 目标(可在配网页覆盖并持久化)
char        g_sta_pass[64] = STA_PASS;
Preferences g_prefs;
WebServer   httpServer(80);
bool        g_softap_on   = false;   // 兜底热点是否已开(保存凭据后需关闭)

// Web 配置页：通过设备所在局域网 IP（或上位机 --auto 发现）访问。
void handle_web_root() {
  // 预分配缓冲 + 逐段 +=: 原写法一条 ~1.7KB 的 String 链式表达式会在内部反复
  // realloc/释放, 且全部动态值散落在链中, 在 loopTask 上造成堆碎片。
  // 现在页面缓冲一次性分配, 每行只产生一个小的临时对象。
  String html; html.reserve(2200);
  html += "<!DOCTYPE html><html lang='zh'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>OV2640 摄像头配置</title>"
    "<style>body{font-family:sans-serif;max-width:520px;margin:40px auto;padding:0 16px}"
    "input{width:100%;padding:10px;margin:6px 0 14px;box-sizing:border-box;font-size:16px}"
    "button{width:100%;padding:12px;background:#2f6fed;color:#fff;border:0;font-size:17px;border-radius:6px}"
    "table{width:100%;border-collapse:collapse}td{padding:6px 0}label{color:#555}</style></head><body>"
    "<h2>OV2640 摄像头配置</h2><table>";
  html += "<tr><td>STA 状态</td><td>" + String(WiFi.status() == WL_CONNECTED ? "已连接" : "未连接")
          + " / " + String(g_sta_ssid) + "</td></tr>";
  html += "<tr><td>设备ID</td><td>" + String(g_dev_id) + "</td></tr>";
  html += "<tr><td>中继</td><td>" + String(g_relay_host[0] ? String(g_relay_host) : String("未配置"))
          + (g_relay_online ? " <b>(在线)</b>" : "") + "</td></tr></table>";
  html += "<hr><h3>设置摄像头要连接的目标 WiFi</h3><form action='/save' method='get'>";
  html += "<label>WiFi 名称 (SSID)</label><input name='ssid' value='" + String(g_sta_ssid) + "'>";
  html += "<label>密码</label><input type='password' name='pass' value='" + String(g_sta_pass) + "'>";
  html += "<hr><h3>入网标识 / 跨网中继(可选)</h3>";
  html += "<label>设备ID (局域网发现与中继注册用, 留空自动)</label>"
          "<input name='id' value='" + String(g_dev_id) + "'>";
  html += "<label>中继服务端 IP:端口 (如 1.2.3.4:8000, 留空=仅局域网)</label>"
          "<input name='relay' value='" + String(g_relay_host) + "'>";
  html += "<label>中继鉴权口令 (与服务器启动参数一致, 留空=不鉴权)</label>"
          "<input name='token' value='" + String(g_relay_token) + "'>";
  html += "<button type='submit'>保存并连接</button></form>"
          "<p style='color:#888'>保存后摄像头将尝试连接该 WiFi。配置页通过目标网络的局域网 IP 访问"
          "(可在路由器管理页或上位机 --auto 发现)，SoftAP 已彻底关闭。</p></body></html>";
  httpServer.send(200, "text/html; charset=utf-8", html);
}

void handle_web_save() {
  String _ssid = httpServer.hasArg("ssid") ? httpServer.arg("ssid") : "";
  String _pass = httpServer.hasArg("pass") ? httpServer.arg("pass") : "";
  String _name = httpServer.hasArg("id")   ? httpServer.arg("id")   : "";
  String _relay= httpServer.hasArg("relay")? httpServer.arg("relay"): "";
  String _token= httpServer.hasArg("token")? httpServer.arg("token"): "";
  _ssid.trim();
  if (_ssid.isEmpty()) {
    httpServer.send(200, "text/html; charset=utf-8",
        "<html><meta charset='utf-8'><body>SSID 不能为空，<a href='/'>返回</a></body></html>");
    return;
  }
  if (_name.isEmpty()) _name = default_dev_id();
  _relay.trim();
  _token.trim();
  g_prefs.begin("wifi", false);
  g_prefs.putString("ssid", _ssid);
  g_prefs.putString("pass", _pass);
  g_prefs.putString("id",   _name);
  g_prefs.putString("relay", _relay);
  g_prefs.putString("token", _token);
  g_prefs.end();

  _ssid.toCharArray(g_sta_ssid, sizeof(g_sta_ssid));
  _pass.toCharArray(g_sta_pass, sizeof(g_sta_pass));
  _name.toCharArray(g_dev_id,   sizeof(g_dev_id));
  if (_name != String(g_dev_name)) _name.toCharArray(g_dev_name, sizeof(g_dev_name));
  _relay.toCharArray(g_relay_host, sizeof(g_relay_host));
  _token.toCharArray(g_relay_token, sizeof(g_relay_token));

  httpServer.send(200, "text/html; charset=utf-8",
      String("<html><meta charset='utf-8'><meta http-equiv='refresh' content='2;url=/'>"
             "<body>已保存目标 WiFi <b>" + _ssid + "</b>，正在连接，2 秒后刷新…</body></html>"));

  // 立即按新目标重连 STA；成功则摄像头入网。
  // 凭据写入后关掉兜底热点把射频还给 STA; 若连不上, 兜底热点会保留以便再次配置。
  if (g_softap_on) {
    WiFi.softAPdisconnect(true);
    g_softap_on = false;
    WiFi.mode(WIFI_STA);
    Dbg.println("[WiFi] 已写入凭据, 兜底热点关闭");
  }
  WiFi.disconnect();
  delay(50);
  WiFi.begin(g_sta_ssid, g_sta_pass);
}

// 默认设备注册 id: cam-<MAC 尾 4 字节>
String default_dev_id() {
  uint8_t m[6]; WiFi.macAddress(m);
  char b[32];
  snprintf(b, sizeof(b), "cam-%02X%02X%02X%02X", m[2], m[3], m[4], m[5]);
  return String(b);
}
