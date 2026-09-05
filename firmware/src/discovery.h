/*
 * 局域网自动发现: 每 3s 广播 "OV2640|<name>|<ip>", 上位机 --auto 监听 UDP 9999。
 */
#pragma once
void disc_announce();
void disc_task(void *arg);
