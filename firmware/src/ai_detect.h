/*
 * 人体检测(Person Detection) 模块。
 * 基于 TensorFlowLite_ESP32 新版 person_detection 例程, 模型为 signed int8 量化:
 *   输入  96x96 灰度(int8, 由 uint8 灰度 XOR 0x80 得到)
 *   输出  2 类(person=1 / not_person=0), 经 zero_point/scale 还原为 float 置信度
 * 模型权重随 .cpp 编译进 flash(~1.9MB), tensor arena 120KB 放内部 SRAM。
 *
 * 集成策略(阶段 1): 只提供"喂一帧灰度图 -> 返回是否有人 + 置信度"的纯推理接口,
 * 不主动切摄像头。采集/分时复用由上层(main.cpp)在阶段 2 调度。
 */
#pragma once
#include <stdint.h>

// 模型输入尺寸(与训练时一致, 改模型需同步改)
#define AI_IMG_W      96
#define AI_IMG_H      96
#define AI_IMG_CH     1          // 灰度

// 推理结果
struct ai_result_t {
  bool  ok;                      // 本次推理是否成功
  bool  person;                  // 是否判定为"人"
  float person_score;            // 人置信度(已还原为 0.0~1.0)
  float not_person_score;        // 非人置信度(0.0~1.0)
};

// 初始化模型与解释器(需在 esp_camera 就绪后调用一次)。失败返回 false。
bool ai_detect_init();

// 喂一帧 96x96 灰度图(uint8, 0=黑 255=白)做一次推理。
// 返回结果; 失败时 ok=false。
ai_result_t ai_detect_once(const uint8_t *gray96);

// 判定阈值: person_score >= 该值才认为"有人"(0.0~1.0)。可在运行时调整。
extern float g_ai_person_thresh;
// 连续确认帧数: 连续 N 帧判定有人才触发一次报警(防抖)。
extern uint8_t g_ai_confirm_frames;
