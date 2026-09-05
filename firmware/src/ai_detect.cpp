/*
 * 人体检测(Person Detection) 实现。
 * 移植自 TensorFlowLite_ESP32 examples/person_detection (新版, signed int8):
 *   - 模型: person_detect_model_data (96x96 灰度输入, 2 类输出)
 *   - 注册 5 个 op: AveragePool2D / Conv2D / DepthwiseConv2D / Reshape / Softmax
 *   - tensor arena 120KB (81KB + ESP32-S3 scratch 39KB), 放内部 SRAM
 */
#include "ai_detect.h"

#include <TensorFlowLite_ESP32.h>

#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "esp_heap_caps.h"

// 模型权重(编译进 flash)。此头由例程提供: 声明 g_person_detect_model_data。
#include "person_detect_model_data.h"

// 类别常量(与模型训练一致, 新版 2 类)
#define AI_K_CATEGORIES     2
#define AI_K_PERSON_IDX     1
#define AI_K_NOT_PERSON_IDX 0

// tensor arena: 81KB 基础 + ESP32-S3 scratch 39KB
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define AI_SCRATCH_BYTES   (39 * 1024)
#else
#define AI_SCRATCH_BYTES   0
#endif
#define AI_ARENA_BYTES     (81 * 1024 + AI_SCRATCH_BYTES)

// 解释器相关(static 生命周期)
static tflite::MicroErrorReporter  s_error_reporter;
static const tflite::Model        *s_model = nullptr;
static tflite::MicroInterpreter   *s_interpreter = nullptr;
static TfLiteTensor               *s_input = nullptr;
static uint8_t                    *s_tensor_arena = nullptr;

// 可调参数(定义于本文件, 声明在 ai_detect.h)
float   g_ai_person_thresh    = 0.6f;  // 人置信度阈值(0.0~1.0)
uint8_t g_ai_confirm_frames   = 1;     // 连续确认帧数(阶段 3 调优用, 现为 1)

bool ai_detect_init() {
  // 从数据映射模型(仅指针映射, 无拷贝/解析)
  s_model = tflite::GetModel(g_person_detect_model_data);
  if (s_model == nullptr) {
    TF_LITE_REPORT_ERROR(&s_error_reporter, "AI: GetModel 失败");
    return false;
  }
  if (s_model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(&s_error_reporter, "AI: 模型 schema v%d != 库支持 v%d",
                         s_model->version(), TFLITE_SCHEMA_VERSION);
    return false;
  }

  // tensor arena 放内部 SRAM(S3 scratch 需要在内部 RAM)
  if (s_tensor_arena == nullptr) {
    s_tensor_arena = (uint8_t*)heap_caps_malloc(AI_ARENA_BYTES,
                                                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (s_tensor_arena == nullptr) {
    TF_LITE_REPORT_ERROR(&s_error_reporter, "AI: tensor arena 分配失败(%d B)",
                         AI_ARENA_BYTES);
    return false;
  }

  // 只注册本模型用到的 5 个 op, 省代码空间
  static tflite::MicroMutableOpResolver<5> resolver;
  resolver.AddAveragePool2D();
  resolver.AddConv2D();
  resolver.AddDepthwiseConv2D();
  resolver.AddReshape();
  resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
      s_model, resolver, s_tensor_arena, AI_ARENA_BYTES, &s_error_reporter);
  s_interpreter = &static_interpreter;

  if (s_interpreter->AllocateTensors() != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(&s_error_reporter, "AI: AllocateTensors 失败");
    return false;
  }

  s_input = s_interpreter->input(0);
  if (s_input == nullptr) {
    TF_LITE_REPORT_ERROR(&s_error_reporter, "AI: 无输入张量");
    return false;
  }
  return true;
}

ai_result_t ai_detect_once(const uint8_t *gray96) {
  ai_result_t r;
  r.ok = false;
  r.person = false;
  r.person_score = 0.0f;
  r.not_person_score = 0.0f;

  if (s_interpreter == nullptr || s_input == nullptr || gray96 == nullptr) {
    return r;
  }

  // uint8 灰度(0..255) -> signed int8(XOR 0x80)
  int8_t *in = s_input->data.int8;
  for (int i = 0; i < AI_IMG_W * AI_IMG_H * AI_IMG_CH; i++) {
    in[i] = (int8_t)(gray96[i] ^ 0x80);
  }

  if (s_interpreter->Invoke() != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(&s_error_reporter, "AI: Invoke 失败");
    return r;
  }

  TfLiteTensor *out = s_interpreter->output(0);
  if (out == nullptr || out->dims->size < 1 || out->dims->data[0] < AI_K_CATEGORIES) {
    return r;
  }

  // int8 量化输出 -> float(按模型输出 zero_point/scale 还原)
  r.person_score =
      (out->data.int8[AI_K_PERSON_IDX] - out->params.zero_point) * out->params.scale;
  r.not_person_score =
      (out->data.int8[AI_K_NOT_PERSON_IDX] - out->params.zero_point) * out->params.scale;

  r.ok = true;
  r.person = (r.person_score >= g_ai_person_thresh);
  return r;
}
