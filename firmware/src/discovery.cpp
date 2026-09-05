/*
 * 局域网自动发现实现(自 main.cpp 拆出)。
 */
#include "globals.h"
#include "discovery.h"

static WiFiUDP  disc_udp;

void disc_announce() {
  if (WiFi.status() != WL_CONNECTED) return;
  IPAddress ip = WiFi.localIP();
  char msg[128];
  snprintf(msg, sizeof(msg), "OV2640|%s|%d.%d.%d.%d",
           g_dev_name, ip[0], ip[1], ip[2], ip[3]);
  disc_udp.beginPacket(IPAddress(255, 255, 255, 255), DISCOVERY_PORT);
  disc_udp.write((const uint8_t*)msg, strlen(msg));
  disc_udp.endPacket();
}

void disc_task(void *arg) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) disc_announce();
    delay(3000);
  }
}
