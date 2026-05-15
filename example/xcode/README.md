# 走马观碑电机控制模块

这套代码按“视觉给状态，电机生成 PWM”的分工写：

- 视觉侧只需要提供 `VisionInfo`：`error`、`road_type`、`line_lost`、`need_slow`、`need_stop`。
- 电机侧先用循迹 PD 算转向量，再按路况生成基础速度，最后用左右轮编码器 PID 闭环生成带符号 `pwm_offset`，并映射为最终 `0..10000` duty。
- 默认引脚只是占位，后面直接改 `default_motor_control_config()` 里的 `MotorPins`。

## 文件说明

- `motor_control.hpp`：结构体、配置、PID 类、电机控制类声明。
- `motor_control.cpp`：PD + 编码器 PID + DRV8701 左右 ATIM PWM/使能输出实现。
- `motor_control_demo.cpp`：接入示例，展示如何每 10ms 调一次 `motor.update()` 并打印 duty。

## 接入方式

把这 3 个文件复制到工程的 `user_app` 目录，或把当前目录加入 CMake 的源码和头文件路径。当前工程 `main/CMakeLists.txt` 已经包含 `../user_app`，所以推荐放到 `user_app`。

在 `main.cpp` 中调用示例：

```cpp
void zmg_motor_control_demo(void);

int main()
{
    zmg_motor_control_demo();
    return 0;
}
```

如果要接视觉线程，把 `motor_control_demo.cpp` 中的 `get_vision_info_from_camera()` 替换为你的视觉结果读取函数即可。

## 常用可调参数

这些字段都在 `MotorControlConfig` 里，默认值集中写在 `default_motor_control_config()`：

- `straight_speed`：直道基础速度。想整体提高或降低直道“基础速度”，优先改这个字段。
- `curve_speed`：弯道基础速度。弯道冲出去、摆动大或转不过来，通常先降低这个字段。
- `slow_ratio`：慢速比例。视觉侧给 `need_slow=true` 时，当前基础速度会再乘 `slow_ratio`。
- `cross_speed`：十字路口基础速度。
- `ramp_speed`：坡道基础速度。
- `max_target_speed`：左右轮目标速度上限，用来限制最高目标速度。
- `max_reverse_speed`：最大反向目标速度，用来限制差速转弯时单侧倒转速度。
- `min_run_pwm_offset`：最小运行 PWM 偏移，用来克服电机静摩擦。
- `max_pwm_offset_from_mid`：PWM 偏移限幅，最终 duty 会被限制在 `5000±max_pwm_offset_from_mid`。
- `speed_to_pwm_gain`：速度前馈换算为 PWM 偏移的增益。
- `turn_pd.kp` / `turn_pd.kd`：循迹转向 PD 参数。`kp` 影响转向力度，`kd` 用来抑制左右摆动。
- `left_speed_pid` / `right_speed_pid`：左右轮编码器速度 PID 参数。常调 `kp`、`ki`、`kd`、`integral_limit`、`output_limit`。

重点：如果只是想改基础速度，直道改 `straight_speed`，弯道改 `curve_speed`；需要临时慢速时让视觉侧置 `need_slow=true`，控制代码会把当前基础速度乘以 `slow_ratio`。

## 调试输出变量名

`motor.debug()` 返回 `MotorControlDebug`，demo 里已经打印了主要字段：

- `base_target_speed`：按路况和 `need_slow` 选出来的基础目标速度。
- `turn_target_speed`：循迹 PD 输出的转向速度差。
- `left_target_speed` / `right_target_speed`：左右轮最终目标速度。
- `left_measured_speed` / `right_measured_speed`：左右编码器采集并换算出的速度反馈值；demo 输出里显示为 `encL` / `encR`。
- `base_pwm_offset`：基础速度对应的前馈 PWM 偏移，仅用于观察。
- `left_pwm_offset` / `right_pwm_offset`：左右轮 PID 后的最终 PWM 偏移。
- `left_duty` / `right_duty`：左右电机最终 duty，`5000` 是停车中点。
- `lost_line_count`：连续丢线计数。

## DRV8701 PWM 模型

实测这块驱动板在使能脚为高电平 3.3V 时：

- `duty = 5000`，也就是 50%，电机停转。
- `duty < 5000` 时正转，越接近 `0` 正转越快。
- `duty > 5000` 时反转，越接近 `10000` 反转越快。

代码内部仍使用正负目标速度和正负控制量。映射关系是：

- 正向控制量 `pwm_offset > 0` -> `duty = 5000 - pwm_offset`。
- 负向控制量 `pwm_offset < 0` -> `duty = 5000 - pwm_offset`，因此 duty 大于 5000。
- `pwm_offset = 0` -> `duty = 5000` 停车。

默认 `config.max_pwm_offset_from_mid = 2500`，最终 duty 被限制在 `2500..7500`。代码还额外保留端点保护，不会输出 `0%` 或 `100%` 满转。调试时建议先设成 `1000..1800`，确认方向、急停和温升正常后，再逐步增大，默认不要超过 `2500` 或 `3000`。

## 停车策略

`stop()`、`safe_shutdown()`、`need_stop`、`RoadType::FINISH`、丢线超时都会让左右两路输出 `duty = 5000`，并保持 ENABLE 高电平。对这块板子来说，停车靠 50% duty，不靠 `duty = 0`，也不靠简单拉低 ENABLE。

注意底层 `ls_pwm`/`ls_atim_pwm`/`ls_gtim_pwm` 的默认析构和 cleanup 行为不适合直接接这块 DRV8701 板：`PWM_POL_INV` 时 cleanup 会先写 `duty = 0` 再 disable，而本硬件会把 `duty = 0` 解释成最大正转。这就是“程序停止后突然满转”的常见原因。当前 `MotorController` 使用中点保持型 ATIM PWM 包装，构造时从 `5000` 开始，退出前调用 `safe_shutdown()`，并避免正常析构触发 `ls_atim_pwm::~ls_atim_pwm()` 的危险 cleanup。

如果 ENABLE 拉低后 PWM 引脚被释放、被下拉/上拉成危险电平，不能把拉低 ENABLE 当作安全停车。优先保证 PWM 引脚持续输出 50% duty，并保持 ENABLE 高电平让驱动板按中点停转。

实车调试必须先架空车轮，确认 `need_stop=true`、Ctrl+C 退出、异常重启时左右轮都不会转，再落地测试。如果某一路，尤其是车头朝前左电机，在程序停止后仍满转，优先检查：

- 该路 PWM 引脚是否在进程退出、复用释放或 disable 后被外部下拉/上拉到 0%/100%。
- 该路 ENABLE 是否和右路不同，或被硬件默认拉成了驱动板会动作的状态。
- 左右 PWM/ENABLE 通道是否接反，默认左路是 `ATIM_PWM1_PIN82` + `PIN_72`，右路是 `ATIM_PWM2_PIN83` + `PIN_73`。
- 驱动板输入端是否需要硬件默认中点/保护电路；更可靠的硬件策略是让 PWM 丢失时不会落到 0% 满正转。
- 调试阶段如果硬件无法保证安全默认态，应让控制程序持续运行并保持 `duty = 5000`，不要让 PWM 被库 cleanup 或系统退出流程接管。

## 方向约定

- `error > 0` 默认表示需要向右修正，此时左轮目标速度增加、右轮目标速度减少。
- 如果实车发现越修越偏，把 `config.positive_error_turn_right` 改为 `false`。
- 差速转向允许某一侧进入反转，反向目标速度由 `config.max_reverse_speed` 限制，默认 `4.0f`。
- 如果某个电机实际正反方向相反，改对应的 `config.left.motor_reverse` 或 `config.right.motor_reverse`。这只会反转 PWM 偏移符号，不使用 DIR GPIO。
- 如果某个编码器前进时速度为负，改对应的 `encoder_reverse`。

## 推荐调参顺序

1. 架空车轮，把 `max_pwm_offset_from_mid` 先设为 `1000..1800`，确认 `need_stop=true` 时 `left_duty/right_duty` 都是 `5000`。
2. 给小目标速度，确认正向时 duty 小于 `5000`、左右轮“前进”方向正确、编码器反馈为正数。
3. 如果电机方向反了，优先改 `motor_reverse`；如果编码器符号反了，改 `encoder_reverse`。
4. 只调速度 PID：先 `ki=0,kd=0`，增大 `kp` 到能跟随但不明显振荡；再少量加 `ki` 消除稳态误差；最后小量加 `kd` 抑制超调。
5. 上地低速调循迹 PD：先只调 `turn_pd.kp`，能转弯后再加 `turn_pd.kd` 抑制左右摆。
6. 再按赛道把 `straight_speed`、`curve_speed`、`slow_ratio`、`lost_line_stop_count`、`max_reverse_speed` 和 `max_pwm_offset_from_mid` 提高或降低。

## DRV8701 驱动板连接

当前代码保持“左 ATIM PWM、左 ENABLE、右 ATIM PWM、右 ENABLE、左右编码器”的接口，不恢复 DIR 引脚。电机 PWM 已改用 `ls_atim_pwm`，编码器采集仍使用 `ls_encoder_pwm`：

- `config.left.pwm_pin`：左电机 ATIM PWM，类型是 `atim_pwm_pin_t`，默认 `ATIM_PWM1_PIN82`。
- `config.left.enable_pin`：左电机 ENABLE；运行和停车都保持高电平，停车靠 50% duty。
- `config.right.pwm_pin`：右电机 ATIM PWM，类型是 `atim_pwm_pin_t`，默认 `ATIM_PWM2_PIN83`。
- `config.right.enable_pin`：右电机 ENABLE；运行和停车都保持高电平，停车靠 50% duty。
- `config.left.encoder_pulse_pin` / `config.right.encoder_pulse_pin`：编码器脉冲输入，仍然必须使用 `ENC_PWMx_PINxx`，不要填 `ATIM_PWMx_PINxx`。

`MotorPins` 配置 `pwm_pin`、`enable_pin`、`encoder_pulse_pin`、`encoder_dir_pin`、`motor_reverse`、`encoder_reverse`。如果你的硬件使能脚固定拉高，可把对应 `enable_pin` 设为 `PIN_INVALID`。
