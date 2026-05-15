#ifndef DET_DIFF_PD_CONTROLLER_HPP
#define DET_DIFF_PD_CONTROLLER_HPP

#include "image.hpp"

// 中线 PD 默认参数。实际调车时建议在 main 中覆盖 cfg.kp / cfg.kd。
#define DET_DIFF_PD_KP (4.0f)   // 比例系数：越大，车对当前偏差修正越快
#define DET_DIFF_PD_KD (1.5f)   // 微分系数：越大，越能抑制摆动和过冲

typedef struct {
  float kp;             // P：当前中线误差增益，可在 main 中直接修改
  float kd;             // D：误差变化量增益，可在 main 中直接修改
  float output_limit;   // turn_pwm 限幅，防止差速过大
  float pwm_min;        // 左右轮最终 PWM 下限
  float pwm_max;        // 左右轮最终 PWM 上限
  float error_deadband; // 误差死区，小误差不修正，减少直道抖动
  uint8 invert_error;   // 置 1 时反向误差方向，便于实车方向快速修正
} DetDiffPdConfig;

typedef struct {
  float last_error; // 上一次中线误差
  float det_error;  // 当前中线误差
  uint8 has_last;   // 第一次运行时不计算 D 项尖峰
} DetDiffPdState;

typedef struct {
  float det_error;      // 中线误差：加权中线 - 图像中心
  float diff_error;     // 本周期误差变化量：error - last_error
  float turn_pwm;       // PD 算出的差速转向量
  float pwm_left;       // 左轮 PWM
  float pwm_right;      // 右轮 PWM
  int weighted_center;  // GetDet() 算出的加权中线 x
} DetDiffPdOutput;

// main 中推荐用法：
// DetDiffPd_SetDefaultConfig(&pd_cfg);
// pd_cfg.kp = 4.0f;
// pd_cfg.kd = 1.5f;
// ImageProcess();  // 内部会调用 GetDet() 更新 ImageStatus.Det_True
// DetDiffPd_UpdateCenterLinePwm(&pd_cfg, &pd_state, base_pwm, &pd_out);
#ifdef __cplusplus
extern "C" {
#endif

void DetDiffPd_SetDefaultConfig(DetDiffPdConfig *cfg);
void DetDiffPd_Reset(DetDiffPdState *state);
float DetDiffPd_GetDetError(const DetDiffPdConfig *cfg);
void DetDiffPd_UpdateCenterLinePwm(const DetDiffPdConfig *cfg,
                                   DetDiffPdState *state,
                                   float base_pwm,
                                   DetDiffPdOutput *out);

#ifdef __cplusplus
}
#endif

#endif
