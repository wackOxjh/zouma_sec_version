#ifndef ZMG_MOTOR_CONTROL_HPP
#define ZMG_MOTOR_CONTROL_HPP

#include <stdint.h>

#include "lq_reg_gpio.hpp"
#include "lq_reg_atim_pwm.hpp"
#include "lq_reg_pwm_encoder.hpp"

namespace zmg
{

enum class RoadType
{
    STRAIGHT = 0,
    CURVE,
    CROSS,
    RAMP,
    START,
    FINISH,
};

struct VisionInfo
{
    float error;          // 视觉给出的中线偏差，单位由视觉侧决定，正负方向可在 config 中反向
    RoadType road_type;   // 路况
    bool line_lost;       // 是否丢线
    bool need_slow;       // 是否需要降速
    bool need_stop;       // 是否需要停车
};

struct PidParam
{
    float kp;              // 用户可调：比例系数，响应不够就加大，振荡明显就减小
    float ki;              // 用户可调：积分系数，用于消除稳态误差，过大容易累积振荡
    float kd;              // 用户可调：微分系数，用于抑制超调和摆动，过大容易放大噪声
    float integral_limit;  // 用户可调：积分限幅，防止积分累积过多
    float output_limit;    // 用户可调：PID/PD 输出限幅，限制单个控制器最大修正量
};

struct MotorPins
{
    atim_pwm_pin_t pwm_pin;
    gpio_pin_t enable_pin;
    ls_enc_pwm_pin_t encoder_pulse_pin;
    gpio_pin_t encoder_dir_pin;
    bool motor_reverse;
    bool encoder_reverse;
};

struct MotorControlConfig
{
    MotorPins left;    // 硬件配置：左电机 PWM/ENABLE/编码器引脚及方向反转
    MotorPins right;   // 硬件配置：右电机 PWM/ENABLE/编码器引脚及方向反转

    uint32_t pwm_freq_hz;     // 用户可调：PWM 频率，DRV8701 常用 10k-25kHz
    float control_period_s;   // 用户可调：控制周期，必须和实际 update() 调用周期一致

    float straight_speed;         // 用户最常改：直道基础速度；想提高/降低基础速度优先改这里
    float curve_speed;            // 用户常改：弯道基础速度；进弯太快就降低这里
    float cross_speed;            // 用户可调：十字路口基础速度
    float ramp_speed;             // 用户可调：坡道基础速度
    float slow_ratio;             // 用户可调：need_slow=true 时基础速度乘以该比例
    float lost_line_speed;        // 用户可调：丢线搜索时前进速度
    float lost_line_turn_speed;   // 用户可调：丢线搜索时差速转向速度
    uint32_t lost_line_stop_count; // 用户可调：连续丢线多少个控制周期后强制停车

    float max_target_speed;          // 用户可调：左右轮目标速度上限，限制最高目标速度
    float max_reverse_speed;         // 用户可调：左右轮最大反向目标速度
    int min_run_pwm_offset;          // 用户可调：克服静摩擦的最小 PWM 偏移
    int max_pwm_offset_from_mid;     // 用户可调：PWM 偏移限幅，最终 duty 限制在 5000±该值
    float speed_to_pwm_gain;         // 用户可调：速度前馈到 PWM 偏移的换算增益

    bool positive_error_turn_right;  // 用户可调：视觉 error 正方向约定，方向反了就取反

    PidParam turn_pd;          // 用户可调：循迹转向 PD，主要调 kp/kd
    PidParam left_speed_pid;   // 用户可调：左轮编码器速度 PID
    PidParam right_speed_pid;  // 用户可调：右轮编码器速度 PID
};

struct MotorControlDebug
{
    float base_target_speed;      // 当前按路况选出的基础目标速度
    float turn_target_speed;      // 循迹 PD 输出的转向速度差
    float left_target_speed;      // 左轮最终目标速度
    float right_target_speed;     // 右轮最终目标速度
    float left_measured_speed;    // 左编码器采集/换算出的速度反馈值，打印时可记为 encL
    float right_measured_speed;   // 右编码器采集/换算出的速度反馈值，打印时可记为 encR
    int base_pwm_offset;          // 基础速度对应的前馈 PWM 偏移，仅用于调试观察
    int left_pwm_offset;          // 左轮 PID 后的最终 PWM 偏移
    int right_pwm_offset;         // 右轮 PID 后的最终 PWM 偏移
    int left_duty;                // 左电机最终 duty，5000 为停车中点
    int right_duty;               // 右电机最终 duty，5000 为停车中点
    uint32_t lost_line_count;     // 当前连续丢线计数
};

MotorControlConfig default_motor_control_config(void);

class PidController
{
public:
    PidController();
    explicit PidController(const PidParam& param);

    void set_param(const PidParam& param);
    void reset(void);
    float update(float target, float feedback, float dt_s);

private:
    PidParam param_;
    float integral_;
    float last_error_;
    bool has_last_error_;
};

class MotorController
{
public:
    explicit MotorController(const MotorControlConfig& config = default_motor_control_config());
    ~MotorController();

    void reset(void);
    void update(const VisionInfo& vision);
    void stop(void);
    void safe_shutdown(void);

    // char set_roadtype();
    void set_straight_speed(float speed);       //外部调用：motor.set_straight_speed(6.0f);
    void set_roadtype_speed(float speed, const VisionInfo& vision);
    void set_Motor_PID_dynamic(float p_param, float i_param, float d_param );
    void set_Vision_PID_dynamic(float vision_p, float vision_i, float vision_d );

    const MotorControlDebug& debug(void) const;

private:
    float select_base_speed(const VisionInfo& vision);
    float calc_turn_speed(const VisionInfo& vision);
    int calc_wheel_pwm_offset(float target_speed, float measured_speed, PidController& pid);
    int pwm_offset_to_duty(int pwm_offset, bool motor_reverse) const;
    void apply_motor(ls_atim_pwm* pwm, ls_gpio* enable, int pwm_duty);
    void apply_motor_stop(ls_atim_pwm* pwm, ls_gpio* enable, int pwm_duty, bool enable_output);

private:
    MotorControlConfig config_;

    ls_atim_pwm* left_pwm_;
    ls_atim_pwm* right_pwm_;
    ls_gpio* left_enable_;
    ls_gpio* right_enable_;
    ls_encoder_pwm left_encoder_;
    ls_encoder_pwm right_encoder_;

    PidController turn_pd_;
    PidController left_speed_pid_;
    PidController right_speed_pid_;

    float last_valid_error_;
    uint32_t lost_line_count_;
    MotorControlDebug debug_;
};

} // namespace zmg

#endif // ZMG_MOTOR_CONTROL_HPP
