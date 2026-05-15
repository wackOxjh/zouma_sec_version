#include "motor_control.hpp"

namespace zmg
{

static const int MOTOR_DUTY_MIN = 0;
static const int MOTOR_DUTY_MID = 5000;
static const int MOTOR_DUTY_MAX = 10000;
static const int MOTOR_DUTY_ENDPOINT_MARGIN = 100;

class SafeMotorAtimPwm : public ls_atim_pwm
{
public:
    SafeMotorAtimPwm(atim_pwm_pin_t pin, uint32_t freq_hz)
        : ls_atim_pwm(pin, freq_hz, MOTOR_DUTY_MID, ATIM_PWM_POL_INV)
    {
    }

    void cleanup(void) override
    {
        // The stock ls_atim_pwm cleanup writes duty=0 before disabling PWM. On this
        // DRV8701 board that is full forward, so keep neutral PWM active instead.
        atim_pwm_set_duty(MOTOR_DUTY_MID);
        atim_pwm_enable();
    }
};

static float limit_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int limit_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int effective_pwm_offset_limit(int configured_limit)
{
    int physical_limit = MOTOR_DUTY_MID - MOTOR_DUTY_ENDPOINT_MARGIN;
    if (ATIM_PWM_DUTY_MAX < MOTOR_DUTY_MAX) {
        physical_limit = ATIM_PWM_DUTY_MAX - MOTOR_DUTY_MID - MOTOR_DUTY_ENDPOINT_MARGIN;
    }
    if (physical_limit < 0) {
        physical_limit = 0;
    }
    return limit_int(configured_limit, 0, physical_limit);
}

static int sign_float(float value)
{
    if (value > 0.0f) {
        return 1;
    }
    if (value < 0.0f) {
        return -1;
    }
    return 0;
}

/*
tip:只负责给一套安全默认值。真正调参时，在外部覆盖。
*/
MotorControlConfig default_motor_control_config(void)
{
    MotorControlConfig config{};    //初始化

    //=============================================== 这些引脚只是示例：电机 PWM 改用 ATIM，编码器仍使用 ENC_PWM 通道采集。
    config.left.pwm_pin = ATIM_PWM1_PIN82;         // 左电机 ATIM PWM 引脚
    config.left.enable_pin = PIN_72;               // 左电机 ENABLE 引脚；固定使能时可设 PIN_INVALID
    config.left.encoder_pulse_pin = ENC_PWM0_PIN64; // 左编码器脉冲输入
    config.left.encoder_dir_pin = PIN_74;          // 左编码器方向输入
    config.left.motor_reverse = false;             // 左电机方向反了就改 true
    config.left.encoder_reverse = true;           // 左编码器前进为负就改 true

    config.right.pwm_pin = ATIM_PWM2_PIN83;        // 右电机 ATIM PWM 引脚
    config.right.enable_pin = PIN_76;              // 右电机 ENABLE 引脚；固定使能时可设 PIN_INVALID       //PIN_73;有问题.
    config.right.encoder_pulse_pin = ENC_PWM1_PIN65; // 右编码器脉冲输入
    config.right.encoder_dir_pin = PIN_75;         // 右编码器方向输入
    config.right.motor_reverse = false;            // 右电机方向反了就改 true
    config.right.encoder_reverse = true;          // 右编码器前进为负就改 true

    config.pwm_freq_hz = 20000;        // 用户可调：DRV8701 常用 10k-25kHz，可按实测噪声和发热调整
    config.control_period_s = 0.01f;   // 用户可调：控制周期；这里表示建议 10ms 调一次 update()

    //=================================== 编码器接口返回值近似为转/秒(rps)，以下速度先作为起跑保守值。
    config.straight_speed = 0.0f;      // 重点可调：直道基础速度；想改“基础速度”优先改这个字段
    config.curve_speed = 4.8f;         // 重点可调：弯道基础速度；弯道不稳就先降低这个字段
    config.cross_speed = 3.5f;         // 用户可调：十字路口基础速度
    config.ramp_speed = 5.0f;          // 用户可调：坡道基础速度
    config.slow_ratio = 0.55f;         // 用户可调：need_slow=true 时，基础速度会乘以该比例
    config.lost_line_speed = 2.0f;     // 用户可调：丢线搜索时的低速前进速度
    config.lost_line_turn_speed = 2.5f; // 用户可调：丢线搜索时的差速转向速度
    config.lost_line_stop_count = 35;  // 用户可调：10ms 周期约 350ms 后丢线停车

    config.max_target_speed = 11.0f;       // 用户可调：左右轮目标速度上限，防止目标速度过大
    config.max_reverse_speed = 4.0f;       // 用户可调：最大反向速度，限制差速转弯时单轮倒转
    config.min_run_pwm_offset = 350;       // 用户可调：最小运行 PWM 偏移，用于克服静摩擦
    config.max_pwm_offset_from_mid = 2500; // 用户可调：PWM 偏移限幅，最终 duty 被限制在 5000±该值
    config.speed_to_pwm_gain = 360.0f;     // 用户可调：速度前馈换算为 PWM 偏移的增益

    // 用户可调：若发现偏差为正时车反而越偏，把这个改成 false。
    config.positive_error_turn_right = false;

    //===================================视觉用户可调：外环循迹 PD。输入视觉 error，输出左右轮目标速度差，通常只调 kp/kd。
    config.turn_pd.kp = 0.035f;          // 转向比例；不够转就加大，来回摆就减小
    config.turn_pd.ki = 0.0f;            // 转向一般不用积分，保持 0
    config.turn_pd.kd = 0.080f;          // 转向微分；用于抑制左右摆动
    config.turn_pd.integral_limit = 0.0f; // ki=0 时该值无效
    config.turn_pd.output_limit = 5.5f;  // 转向速度差限幅，防止单次修正过猛

    // 电机用户可调：内环速度 PID。输入目标 rps 和编码器反馈 rps，输出 PWM 修正量。
    config.left_speed_pid.kp = 4.0f;//220.0f;             // 左轮速度比例
    config.left_speed_pid.ki = 0.1f;//25.0f;              // 左轮速度积分
    config.left_speed_pid.kd = 0.1f;//6.0f;               // 左轮速度微分
    config.left_speed_pid.integral_limit = 120.0f; // 左轮速度积分限幅
    config.left_speed_pid.output_limit = 4000.0f;  // 左轮速度 PID 输出限幅

    config.right_speed_pid = config.left_speed_pid; // 右轮默认沿用左轮 PID；两侧差异明显时可单独覆盖

    return config;
}

PidController::PidController()
    : integral_(0.0f),
      last_error_(0.0f),
      has_last_error_(false)
{
    param_.kp = 0.0f;
    param_.ki = 0.0f;
    param_.kd = 0.0f;
    param_.integral_limit = 0.0f;
    param_.output_limit = 0.0f;
}

PidController::PidController(const PidParam& param)
    : param_(param),
      integral_(0.0f),
      last_error_(0.0f),
      has_last_error_(false)
{
}

void PidController::set_param(const PidParam& param)
{
    param_ = param;
    reset();
}

void PidController::reset(void)
{
    integral_ = 0.0f;
    last_error_ = 0.0f;
    has_last_error_ = false;
}

float PidController::update(float target, float feedback, float dt_s)
{
    const float error = target - feedback;

    if (param_.ki != 0.0f && dt_s > 0.0f) {
        integral_ += error * dt_s;
        if (param_.integral_limit > 0.0f) {
            integral_ = limit_float(integral_, -param_.integral_limit, param_.integral_limit);
        }
    }

    float derivative = 0.0f;
    if (has_last_error_ && dt_s > 0.0f) {
        derivative = (error - last_error_) / dt_s;
    }

    last_error_ = error;
    has_last_error_ = true;

    float output = param_.kp * error + param_.ki * integral_ + param_.kd * derivative;
    if (param_.output_limit > 0.0f) {
        output = limit_float(output, -param_.output_limit, param_.output_limit);
    }
    return output;
}

MotorController::MotorController(const MotorControlConfig& config)
    : config_(config),
      left_pwm_(new SafeMotorAtimPwm(config.left.pwm_pin, config.pwm_freq_hz)),
      right_pwm_(new SafeMotorAtimPwm(config.right.pwm_pin, config.pwm_freq_hz)),
      left_enable_(config.left.enable_pin == PIN_INVALID ? 0 : new ls_gpio(config.left.enable_pin, GPIO_MODE_OUT)),
      right_enable_(config.right.enable_pin == PIN_INVALID ? 0 : new ls_gpio(config.right.enable_pin, GPIO_MODE_OUT)),
      left_encoder_(config.left.encoder_pulse_pin, config.left.encoder_dir_pin),
      right_encoder_(config.right.encoder_pulse_pin, config.right.encoder_dir_pin),
      turn_pd_(config.turn_pd),
      left_speed_pid_(config.left_speed_pid),
      right_speed_pid_(config.right_speed_pid),
      last_valid_error_(0.0f),
      lost_line_count_(0)
{
    reset();
}

MotorController::~MotorController()
{
    safe_shutdown();
    delete left_enable_;
    delete right_enable_;
    left_enable_ = 0;
    right_enable_ = 0;

    // Do not delete left_pwm_/right_pwm_. ls_atim_pwm::~ls_atim_pwm() calls cleanup(),
    // which drives PWM to 0/10000 before disabling and is unsafe for this board.
    // The leaked SafeMotorAtimPwm objects also override cleanup_all() to hold 5000.
}

void MotorController::reset(void)
{
    turn_pd_.reset();
    left_speed_pid_.reset();
    right_speed_pid_.reset();
    last_valid_error_ = 0.0f;
    lost_line_count_ = 0;

    debug_.base_target_speed = 0.0f;
    debug_.turn_target_speed = 0.0f;
    debug_.left_target_speed = 0.0f;
    debug_.right_target_speed = 0.0f;
    debug_.left_measured_speed = 0.0f;
    debug_.right_measured_speed = 0.0f;
    debug_.base_pwm_offset = 0;
    debug_.left_pwm_offset = 0;
    debug_.right_pwm_offset = 0;
    debug_.left_duty = MOTOR_DUTY_MID;
    debug_.right_duty = MOTOR_DUTY_MID;
    debug_.lost_line_count = 0;

    stop();
}

void MotorController::update(const VisionInfo& vision)
{
    if (vision.line_lost) {
        ++lost_line_count_;
    } else {
        lost_line_count_ = 0;
        last_valid_error_ = vision.error;
    }

    const bool force_stop = vision.need_stop ||
        vision.road_type == RoadType::FINISH ||
        (vision.line_lost && lost_line_count_ >= config_.lost_line_stop_count);

    if (force_stop) {
        stop();
        debug_.lost_line_count = lost_line_count_;
        return;
    }

    const float base_speed = select_base_speed(vision);
    const float turn_speed = calc_turn_speed(vision);

    float left_target = base_speed + turn_speed;
    float right_target = base_speed - turn_speed;

    left_target = limit_float(left_target, -config_.max_reverse_speed, config_.max_target_speed);
    right_target = limit_float(right_target, -config_.max_reverse_speed, config_.max_target_speed);

    // 左右编码器采集/换算出的速度反馈值，最终写入 debug.left_measured_speed/right_measured_speed。
    float left_measured = left_encoder_.encoder_get_count();
    float right_measured = right_encoder_.encoder_get_count();
    if (config_.left.encoder_reverse) {
        left_measured = -left_measured;
    }
    if (config_.right.encoder_reverse) {
        right_measured = -right_measured;
    }

    const int left_pwm_offset = calc_wheel_pwm_offset(left_target, left_measured, left_speed_pid_);
    const int right_pwm_offset = calc_wheel_pwm_offset(right_target, right_measured, right_speed_pid_);
    const int left_duty = pwm_offset_to_duty(left_pwm_offset, config_.left.motor_reverse);
    const int right_duty = pwm_offset_to_duty(right_pwm_offset, config_.right.motor_reverse);

    apply_motor(left_pwm_, left_enable_, left_duty);
    apply_motor(right_pwm_, right_enable_, right_duty);

    debug_.base_target_speed = base_speed;
    debug_.turn_target_speed = turn_speed;
    debug_.left_target_speed = left_target;
    debug_.right_target_speed = right_target;
    debug_.left_measured_speed = left_measured;
    debug_.right_measured_speed = right_measured;
    const int max_pwm_offset = effective_pwm_offset_limit(config_.max_pwm_offset_from_mid);
    const int min_run_pwm_offset = limit_int(config_.min_run_pwm_offset, 0, max_pwm_offset);
    debug_.base_pwm_offset = base_speed > 0.0f
        ? limit_int((int)(base_speed * config_.speed_to_pwm_gain) + min_run_pwm_offset,
              0,
              max_pwm_offset)
        : 0;
    debug_.left_pwm_offset = left_pwm_offset;
    debug_.right_pwm_offset = right_pwm_offset;
    debug_.left_duty = left_duty;
    debug_.right_duty = right_duty;
    debug_.lost_line_count = lost_line_count_;
}

void MotorController::stop(void)
{
    apply_motor_stop(left_pwm_, left_enable_, MOTOR_DUTY_MID, false);
    apply_motor_stop(right_pwm_, right_enable_, MOTOR_DUTY_MID, false);

    left_speed_pid_.reset();
    right_speed_pid_.reset();

    debug_.base_target_speed = 0.0f;
    debug_.turn_target_speed = 0.0f;
    debug_.left_target_speed = 0.0f;
    debug_.right_target_speed = 0.0f;
    debug_.base_pwm_offset = 0;
    debug_.left_pwm_offset = 0;
    debug_.right_pwm_offset = 0;
    debug_.left_duty = MOTOR_DUTY_MID;
    debug_.right_duty = MOTOR_DUTY_MID;
}

void MotorController::safe_shutdown(void)
{
    stop();
}

const MotorControlDebug& MotorController::debug(void) const
{
    return debug_;
}

float MotorController::select_base_speed(const VisionInfo& vision)
{
    float speed = config_.straight_speed;

    switch (vision.road_type) {
    case RoadType::CURVE:
        speed = config_.curve_speed;
        break;
    case RoadType::CROSS:
        speed = config_.cross_speed;
        break;
    case RoadType::RAMP:
        speed = config_.ramp_speed;
        break;
    case RoadType::FINISH:
        speed = 0.0f;
        break;
    case RoadType::START:
    case RoadType::STRAIGHT:
    default:
        speed = config_.straight_speed;
        break;
    }

    if (vision.need_slow) {
        speed *= config_.slow_ratio;
    }

    if (vision.line_lost) {
        speed = config_.lost_line_speed;
    }

    return limit_float(speed, 0.0f, config_.max_target_speed);
}

float MotorController::calc_turn_speed(const VisionInfo& vision)
{
    if (vision.line_lost) {
        float search_error = last_valid_error_;
        if (!config_.positive_error_turn_right) {
            search_error = -search_error;
        }

        int search_dir = sign_float(search_error);
        if (search_dir == 0) {
            search_dir = 1;
        }
        return (float)search_dir * config_.lost_line_turn_speed;
    }

    float error = vision.error;
    if (!config_.positive_error_turn_right) {
        error = -error;
    }

    return turn_pd_.update(error, 0.0f, config_.control_period_s);
}

int MotorController::calc_wheel_pwm_offset(float target_speed, float measured_speed, PidController& pid)
{
    if (target_speed > -0.001f && target_speed < 0.001f) {
        pid.reset();
        return 0;
    }

    const int target_sign = sign_float(target_speed);
    const int max_offset = effective_pwm_offset_limit(config_.max_pwm_offset_from_mid);
    const int min_offset = limit_int(config_.min_run_pwm_offset, 0, max_offset);
    float pwm_offset = target_speed * config_.speed_to_pwm_gain;
    pwm_offset += (float)(target_sign * min_offset);
    pwm_offset += pid.update(target_speed, measured_speed, config_.control_period_s);

    // 调速度 PID 时不要允许 PID 把电机反向打过去，否则会前进/后退来回抖。
    if (target_sign > 0 && pwm_offset < 0.0f) {
        pwm_offset = 0.0f;
        lq_log_info("1");
    } else if (target_sign < 0 && pwm_offset > 0.0f) {
        pwm_offset = 0.0f;
        lq_log_info("2");
    }

    return limit_int((int)pwm_offset, -max_offset, max_offset);
}

int MotorController::pwm_offset_to_duty(int pwm_offset, bool motor_reverse) const
{
    const int max_offset = effective_pwm_offset_limit(config_.max_pwm_offset_from_mid);
    pwm_offset = limit_int(pwm_offset, -max_offset, max_offset);
    if (motor_reverse) {
        pwm_offset = -pwm_offset;
    }

    // DRV8701 实测：5000 停车，正向控制量要输出小于 5000 的 duty。
    return limit_int(MOTOR_DUTY_MID - pwm_offset,
        MOTOR_DUTY_MID - max_offset,
        MOTOR_DUTY_MID + max_offset);
}

void MotorController::apply_motor(ls_atim_pwm* pwm, ls_gpio* enable, int pwm_duty)
{
    pwm_duty = limit_int(pwm_duty,
        MOTOR_DUTY_MIN + MOTOR_DUTY_ENDPOINT_MARGIN,
        MOTOR_DUTY_MAX - MOTOR_DUTY_ENDPOINT_MARGIN);
    if (pwm_duty > ATIM_PWM_DUTY_MAX) {
        pwm_duty = ATIM_PWM_DUTY_MAX;
    }

    if (pwm != 0) {
        pwm->atim_pwm_set_duty((uint32_t)pwm_duty);
        pwm->atim_pwm_enable();
    }
    if (enable != 0) {
        enable->gpio_level_set(GPIO_HIGH);
    }
}

void MotorController::apply_motor_stop(ls_atim_pwm* pwm, ls_gpio* enable, int pwm_duty, bool enable_output)
{
    pwm_duty = limit_int(pwm_duty,
        MOTOR_DUTY_MIN + MOTOR_DUTY_ENDPOINT_MARGIN,
        MOTOR_DUTY_MAX - MOTOR_DUTY_ENDPOINT_MARGIN);
    if (pwm_duty > ATIM_PWM_DUTY_MAX) {
        pwm_duty = ATIM_PWM_DUTY_MAX;
    }

    if (pwm != 0) {
        pwm->atim_pwm_set_duty((uint32_t)pwm_duty);
        pwm->atim_pwm_enable();
    }
    if (enable != 0) {
        enable->gpio_level_set(enable_output ? GPIO_HIGH:GPIO_LOW);
    }
}

/*
name：设置调试时的直线速度
param：speed
use：这个 config 只在构造函数里面存在。构造函数执行完以后，config 就没了，类里面留下来的是复制后的 config_。
可以这样理解：
MotorController::MotorController(const MotorControlConfig& config)
这里的 config 是“外面传进来的临时参考”。
MotorControlConfig config_;
这里的 config_ 是“电机控制器自己保存的一份参数”。
所以在 set_straight_speed() 这种成员函数里，应该用：config_.max_target_speed
不能用：config.max_target_speed*/
void MotorController::set_straight_speed(float speed)
{
    if (speed > config_.max_target_speed) {
        speed = config_.max_target_speed;
    }
    config_.straight_speed = speed;
}

/*
name:各路况速度调整
param:速度  路况
application:实时调参时可修改*/
void MotorController::set_roadtype_speed(float speed, const VisionInfo& vision)
{
    config_.straight_speed = speed;

    switch (vision.road_type) {
    case RoadType::CURVE:
        config_.curve_speed = speed;
        break;
    case RoadType::CROSS:
        config_.cross_speed = speed;
        break;
    case RoadType::RAMP:
        config_.ramp_speed = speed;
        break;
    case RoadType::FINISH:
        speed = 0.0f;
        break;
    case RoadType::START:
    case RoadType::STRAIGHT:
    default:
        config_.straight_speed = speed;
        break;
    }
    
    if (speed > config_.max_target_speed)
    {
        speed = config_.max_target_speed;
    }

}

/*
tip：动态修改PID（电机）参数*/
void MotorController::set_Motor_PID_dynamic(float p_param, float i_param, float d_param )
{
    config_.left_speed_pid.kp = p_param;
    config_.left_speed_pid.ki = i_param;
    config_.left_speed_pid.kd = d_param;

    config_.right_speed_pid = config_.left_speed_pid;

    left_speed_pid_.set_param(config_.left_speed_pid);
    right_speed_pid_.set_param(config_.right_speed_pid);
}

/*
tip：动态修改PD（视觉）参数*/
void MotorController::set_Vision_PID_dynamic(float vision_p, float vision_i, float vision_d )
{
    config_.turn_pd.kp = vision_p;
    config_.turn_pd.kd = vision_d;

    config_.turn_pd.ki = 0.0;
    
    turn_pd_.set_param(config_.turn_pd);
}


} // namespace zmg
