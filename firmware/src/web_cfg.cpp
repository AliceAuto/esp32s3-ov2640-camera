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
String      g_scan_cache  = "[]";    // 附近 WiFi 扫描结果缓存(开热点前扫好, 配网页直接读)
bool        g_scan_cached = false;   // 是否已有缓存(避免重复扫描)

// 真正执行一次同步扫描, 结果序列化为 JSON 缓存到 g_scan_cache。
// 必须在 STA 模式(未开热点)时调用; AP 模式与扫描共用单射频, 开热点后扫描会被打断。
static void do_scan() {
  // 扫描前先断开正在进行的 STA 连接尝试(此时可能卡在 NO_AP_FOUND 反复重连),
  // 释放射频再扫, 否则同步扫描可能被连接尝试干扰返回 0。
  WiFi.disconnect(true);
  delay(100);
  int n = WiFi.scanNetworks(false, false);   // 同步阻塞到扫描完成
  Dbg.printf("[扫描] 发现 %d 个网络\n", n);
  if (n < 0) n = 0;
  String out;
  out.reserve(n * 40 + 16);
  out += "[";
  bool first = true;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    Dbg.printf("[扫描]   %d: '%s' rssi=%d enc=%d\n", i, ssid.c_str(),
               WiFi.RSSI(i), (int)WiFi.encryptionType(i));
    if (ssid.isEmpty()) continue;
    String esc;
    for (size_t k = 0; k < ssid.length(); k++) {
      char c = ssid[k];
      if (c == '\\' || c == '"') { esc += '\\'; esc += c; }
      else if (c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); esc += b; }
      else esc += c;
    }
    if (!first) out += ",";
    first = false;
    out += "{\"s\":\"" + esc + "\",\"r\":" + String(WiFi.RSSI(i))
         + ",\"e\":" + String(WiFi.encryptionType(i)) + "}";
  }
  out += "]";
  WiFi.scanDelete();
  g_scan_cache = out;
  g_scan_cached = true;
}

// 开热点前扫描并缓存。仅在未缓存过时真正扫描, 幂等可重复调用。
void wifi_scan_and_cache() {
  if (!g_scan_cached) do_scan();
}

// 配网页 /scan: 返回缓存的附近 WiFi 列表(开热点前已扫好)。
void handle_web_scan() {
  if (!g_scan_cached) wifi_scan_and_cache();   // 兜底: 万一尚未缓存
  httpServer.send(200, "application/json; charset=utf-8", g_scan_cache);
}

// Web 配置页：通过设备所在局域网 IP（或上位机 --auto 发现）访问。
void handle_web_root() {
  // 预分配缓冲 + 逐段 +=: 原写法一条 ~1.7KB 的 String 链式表达式会在内部反复
  // realloc/释放, 且全部动态值散落在链中, 在 loopTask 上造成堆碎片。
  // 现在页面缓冲一次性分配, 每行只产生一个小的临时对象。
  String html; html.reserve(3000);
  html += "<!DOCTYPE html><html lang='zh'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>OV2640 摄像头配置</title>"
    "<style>body{font-family:sans-serif;max-width:520px;margin:40px auto;padding:0 16px}"
    "input,select{width:100%;padding:10px;margin:6px 0 14px;box-sizing:border-box;font-size:16px}"
    "button{width:100%;padding:12px;background:#2f6fed;color:#fff;border:0;font-size:17px;border-radius:6px;margin:4px 0}"
    ".row{display:flex;gap:8px}.row button{width:auto;padding:10px 16px}"
    "table{width:100%;border-collapse:collapse}td{padding:6px 0}label{color:#555}"
    "#list{max-height:180px;overflow:auto;border:1px solid #ccc;border-radius:6px;margin:6px 0 14px}"
    ".ap{padding:10px 12px;border-bottom:1px solid #eee;cursor:pointer;display:flex;justify-content:space-between}"
    ".ap:hover{background:#eef4ff}</style></head><body>"
    "<h2>OV2640 摄像头配置</h2><table>";
  html += "<tr><td>STA 状态</td><td>" + String(WiFi.status() == WL_CONNECTED ? "已连接" : "未连接")
          + " / " + String(g_sta_ssid) + "</td></tr>";
  html += "<tr><td>设备ID</td><td>" + String(g_dev_id) + "</td></tr>";
  html += "<tr><td>中继</td><td>" + String(g_relay_host[0] ? String(g_relay_host) : String("未配置"))
          + (g_relay_online ? " <b>(在线)</b>" : "") + "</td></tr></table>";
  html += "<hr><h3>设置摄像头要连接的目标 WiFi</h3><form action='/save' method='get'>";
  html += "<label>WiFi 名称 (SSID)</label><div class='row'>"
          "<input name='ssid' id='ssid' value='" + String(g_sta_ssid) + "' placeholder='点击下方扫描后选择'>"
          "<button type='button' onclick='scan()'>扫描</button></div>"
          "<div id='list' style='display:none'></div>";
  html += "<label>密码</label><div class='row'>"
          "<input type='password' name='pass' id='pass' value='" + String(g_sta_pass) + "' style='flex:1'>"
          "<button type='button' id='eyebtn' onclick='togglePass()'>显示</button></div>";
  html += "<div class='row'>"
          "<button type='button' id='testbtn' onclick='testConn()'>测试连接</button></div>"
          "<div id='testmsg' style='margin:4px 0 14px;color:#555'></div>";
  html += "<hr><h3>入网标识 / 跨网中继(可选)</h3>";
  html += "<label>设备ID (局域网发现与中继注册用, 留空自动)</label>"
          "<input name='id' value='" + String(g_dev_id) + "'>";
  html += "<label>中继服务端 IP:端口 (如 1.2.3.4:8000, 留空=仅局域网)</label>"
          "<input name='relay' value='" + String(g_relay_host) + "'>";
  html += "<label>中继鉴权口令 (与服务器启动参数一致, 留空=不鉴权)</label>"
          "<input name='token' value='" + String(g_relay_token) + "'>";
  html += "<button type='submit'>保存并连接</button></form>"
          "<p style='color:#888'>保存后摄像头将尝试连接该 WiFi。配置页通过目标网络的局域网 IP 访问"
          "(可在路由器管理页或上位机 --auto 发现)。连不上目标 WiFi 时会自动开启热点 OV2640Cam 供重新配网。</p>"
          "<script>"
          "function scan(){var l=document.getElementById('list');l.innerHTML='扫描中…';l.style.display='block';"
          "fetch('/scan').then(r=>r.json()).then(a=>{l.innerHTML='';"
          "a.sort((x,y)=>y.r-x.r);"
          "a.forEach(w=>{var d=document.createElement('div');d.className='ap';"
          "d.innerHTML='<span>'+w.s+'</span><span style=\"color:#999\">'+w.r+' dBm'+(w.e? ' 🔒':'')+'</span>';"
          "d.onclick=function(){document.getElementById('ssid').value=w.s;l.style.display='none';};"
          "l.appendChild(d);});"
          "if(!a.length)l.innerHTML='未扫描到 WiFi';"
          "}).catch(e=>{l.innerHTML='扫描失败: '+e;});}"
          "function togglePass(){var i=document.getElementById('pass');var b=document.getElementById('eyebtn');"
          "if(i.type==='password'){i.type='text';b.textContent='隐藏';}else{i.type='password';b.textContent='显示';}}"
          "function testConn(){var s=document.getElementById('ssid').value;var p=document.getElementById('pass').value;"
          "var m=document.getElementById('testmsg');var b=document.getElementById('testbtn');"
          "m.innerHTML='测试中…';b.disabled=true;"
          "fetch('/test?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p)).then(r=>r.json()).then(j=>{"
          "if(!j.started){m.innerHTML='<span style=\"color:#d33\">✗ '+j.msg+'</span>';b.disabled=false;return;}"
          "pollTest(m,b);"
          "}).catch(e=>{m.innerHTML='测试请求失败: '+e;b.disabled=false;});}"
          "function pollTest(m,b){setTimeout(function(){"
          "fetch('/test_status').then(r=>r.json()).then(j=>{"
          "if(j.running){m.innerHTML='测试中…';pollTest(m,b);}"
          "else{b.disabled=false;"
          "if(j.ok){m.innerHTML='<span style=\"color:#2e9e4f\">✓ 连接成功, IP: '+j.ip+'</span>';}"
          "else{m.innerHTML='<span style=\"color:#d33\">✗ 连接失败 (status '+j.status+')</span>';}"
          "}}).catch(e=>{m.innerHTML='测试中…(热点重连中)';pollTest(m,b);});"
          "},1200);}"
          "</script></body></html>";
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

// 测试连接状态机(异步): 因为单射频, STA 测试连接会打断热点, 若同步等待并
// 用同一 HTTP 连接返回结果, 连接成功时热点已断 -> 响应回不去 -> 前端卡"测试中"。
// 故 /test 只启动并立即返回, 状态由 test_tick 在 loop 非阻塞推进,
// 前端轮询 /test_status 取结果, 不依赖单次长连接。
static bool     g_test_running = false;
static bool     g_test_ok      = false;
static int      g_test_status  = 0;
static String   g_test_ip      = "";
static String   g_test_ssid, g_test_pass;
static uint32_t g_test_t0      = 0;

// 启动测试: 记录凭据, 立即返回"已开始"
void handle_web_test() {
  String _ssid = httpServer.hasArg("ssid") ? httpServer.arg("ssid") : "";
  String _pass = httpServer.hasArg("pass") ? httpServer.arg("pass") : "";
  _ssid.trim();
  if (_ssid.isEmpty()) {
    httpServer.send(200, "application/json; charset=utf-8",
        "{\"started\":false,\"msg\":\"SSID 不能为空\"}");
    return;
  }
  g_test_ssid = _ssid;
  g_test_pass = _pass;
  g_test_running = true;
  g_test_ok = false;
  g_test_status = 0;
  g_test_ip = "";
  g_test_t0 = millis();
  Dbg.printf("[测试] 启动异步连接 '%s' ...\n", g_test_ssid.c_str());
  WiFi.disconnect();
  delay(50);
  WiFi.begin(g_test_ssid.c_str(), g_test_pass.c_str());
  httpServer.send(200, "application/json; charset=utf-8", "{\"started\":true}");
}

// 状态机: loop 周期调用, 非阻塞推进测试
void test_tick() {
  if (!g_test_running) return;
  if (WiFi.status() == WL_CONNECTED) {
    // 成功
    g_test_running = false;
    g_test_ok = true;
    g_test_ip = WiFi.localIP().toString();
    Dbg.printf("[测试] 成功 ip=%s\n", g_test_ip.c_str());
    // 恢复配网热点(若之前处于配网模式), 手机自动重连后轮询即可取到结果
    WiFi.disconnect();
    if (g_softap_on) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(AP_SSID, AP_PASS);
    }
    return;
  }
  if (millis() - g_test_t0 >= 10000) {
    // 超时失败
    g_test_running = false;
    g_test_ok = false;
    g_test_status = WiFi.status();
    Dbg.printf("[测试] 失败 status=%d\n", g_test_status);
    WiFi.disconnect();
    if (g_softap_on) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(AP_SSID, AP_PASS);
    }
  }
}

// 查询测试结果(前端轮询)
void handle_web_test_status() {
  String json = String("{\"running\":") + (g_test_running ? "true" : "false")
              + ",\"ok\":" + (g_test_ok ? "true" : "false")
              + ",\"status\":" + String(g_test_status)
              + (g_test_ip.length() ? ",\"ip\":\"" + g_test_ip + "\"" : "")
              + "}";
  httpServer.send(200, "application/json; charset=utf-8", json);
}

// 默认设备注册 id: cam-<MAC 尾 4 字节>
String default_dev_id() {
  uint8_t m[6]; WiFi.macAddress(m);
  char b[32];
  snprintf(b, sizeof(b), "cam-%02X%02X%02X%02X", m[2], m[3], m[4], m[5]);
  return String(b);
}
