#include "det_diff_pd_controller.hpp"

// 数值限幅函数。
// 作用：
// 1. 限制 turn_pwm，避免转向差速过大导致车辆甩尾；
// 2. 限制左右轮最终 PWM，避免超过电机驱动允许范围。
static float DetDiffPd_Clamp(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

// 简单绝对值函数，避免额外依赖 math.h。
// 这里用于判断误差是否落在死区 error_deadband 内。
static float DetDiffPd_Abs(float value) {
  return (value < 0.0f) ? -value : value;
}

// 初始化 PD 控制器参数。
// main 中可以先调用这个函数给 cfg 赋默认值，再单独修改 kp、kd 等参数。
// 例如：
//   DetDiffPd_SetDefaultConfig(&pd_cfg);
//   pd_cfg.kp = 4.0f;
//   pd_cfg.kd = 1.5f;
void DetDiffPd_SetDefaultConfig(DetDiffPdConfig *cfg) {
  if (cfg == 0) {
    return;
  }

  cfg->kp = DET_DIFF_PD_KP;       // P 参数：越大转向越积极，过大容易左右摆
  cfg->kd = DET_DIFF_PD_KD;       // D 参数：抑制误差变化，减小入弯过冲和直道抖动
  cfg->output_limit = 8000.0f;    // PD 输出限幅，即最大允许差速 PWM
  cfg->pwm_min = -8000.0f;        // 电机 PWM 下限，根据你的驱动范围修改
  cfg->pwm_max = 8000.0f;         // 电机 PWM 上限，根据你的驱动范围修改
  cfg->error_deadband = 0.0f;     // 默认不加死区；直道抖动时可尝试 1~3 像素
  cfg->invert_error = 0;          // 若实车转向方向反了，把它改成 1
}

// 重置控制器状态。
// 需要在发车前、停车后重新发车、切换控制模式时调用。
// 这样可以清掉 last_error，避免上一次运行留下的 D 项影响下一次启动。
void DetDiffPd_Reset(DetDiffPdState *state) {
  if (state == 0) {
    return;
  }

  state->last_error = 0.0f;
  state->det_error = 0.0f;
  state->has_last = 0;
}

// 读取图像中线误差。
// 注意：GetDet() 在 image.c 中已经根据前瞻点和附近几行做了加权平均，
//      结果保存在 ImageStatus.Det_True。
//
// 本函数只做“中线 x -> 误差”的转换：
//   error = 加权中线 x - 图像中心 x
//
// 按当前方向约定：
//   error > 0：赛道中心在画面右侧，需要向右修正；
//   error < 0：赛道中心在画面左侧，需要向左修正。
//
// 如果接上电机后发现方向正好反了，不用改公式，直接 cfg->invert_error = 1。
float DetDiffPd_GetDetError(const DetDiffPdConfig *cfg) {
  // GetDet() 中的 ImageStatus.Det_True 实际保存的是加权后的赛道中线 x。
  float error = (float)ImageStatus.Det_True - (float)ImageSensorMid;

  if (cfg != 0 && cfg->invert_error) {
    error = -error;
  }

  return error;
}

// 根据中线误差计算左右轮 PWM。
//
// 输入：
//   cfg      ：PD 参数和限幅参数
//   state    ：保存 last_error，用来计算 D 项
//   base_pwm ：基础速度 PWM，通常由 main 根据目标车速给出
//   out      ：输出误差、转向量、左右轮 PWM，方便调试显示
//
// 控制核心：
//   turn_pwm  = kp * error + kd * (error - last_error)
//   left_pwm  = base_pwm - turn_pwm
//   right_pwm = base_pwm + turn_pwm
//
// 因此当 error > 0 时，turn_pwm 通常为正，右轮 PWM 增大、左轮 PWM 减小，
// 差速车会向右修正。
void DetDiffPd_UpdateCenterLinePwm(const DetDiffPdConfig *cfg,
                                   DetDiffPdState *state,
                                   float base_pwm,
                                   DetDiffPdOutput *out) {
  if (cfg == 0 || state == 0 || out == 0) {
    return;
  }

  // 1. 获取当前图像误差。
  float error = DetDiffPd_GetDetError(cfg);
  lq_log_info("当前误差: %.2f", error);

  // 2. 死区处理：误差很小时认为已经在中间，减少直道小幅抖动。
  if (DetDiffPd_Abs(error) < cfg->error_deadband) {
    error = 0.0f;
  }

  // 3. D 项计算：只看误差变化量，不做积分，避免视觉循迹中积分累积失控。
  // 第一次进入控制时没有 last_error，D 项先置 0，避免启动瞬间差速突变。
  float diff_error = state->has_last ? (error - state->last_error) : 0.0f;

  // 4. PD 输出：P 负责根据当前偏差转向，D 负责抑制偏差变化过快。
  float turn_pwm = cfg->kp * error + cfg->kd * diff_error;

  // 5. 限制最大差速，防止急弯或误差跳变时输出过大。
  turn_pwm = DetDiffPd_Clamp(turn_pwm, -cfg->output_limit, cfg->output_limit);

  // 6. 保存调试输出，便于在屏幕或串口查看 error、D 项和 turn_pwm。
  out->det_error = error;
  out->diff_error = diff_error;
  out->turn_pwm = turn_pwm;
  out->weighted_center = ImageStatus.Det_True;

  // 7. 差速分配到左右轮，并再次限制最终 PWM 范围。
  out->pwm_left = DetDiffPd_Clamp(base_pwm - turn_pwm, cfg->pwm_min, cfg->pwm_max);
  out->pwm_right = DetDiffPd_Clamp(base_pwm + turn_pwm, cfg->pwm_min, cfg->pwm_max);

  // 8. 更新状态，供下一周期计算 D 项。
  state->last_error = error;
  state->det_error = error;
  state->has_last = 1;
}
/*
main 中可以这样调用：

DetDiffPdConfig pd_cfg;
DetDiffPdState pd_state;
DetDiffPdOutput pd_out;

DetDiffPd_SetDefaultConfig(&pd_cfg);
DetDiffPd_Reset(&pd_state);

pd_cfg.kp = 4.0f;
pd_cfg.kd = 1.5f;

ImageProcess();
DetDiffPd_UpdateCenterLinePwm(&pd_cfg, &pd_state, base_pwm, &pd_out);

// Set_PWM_L(pd_out.pwm_left);
// Set_PWM_R(pd_out.pwm_right);
*/
