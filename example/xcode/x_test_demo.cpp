#include "lq_all_demo.hpp"
#include "image.hpp"
#include "det_diff_pd_controller.hpp"
#include "motor_control.hpp"

#include <stdio.h>
#include <unistd.h>

/********************************************************************************
 * @file    lq_udp_img_trans_demo.cpp
 * @brief   UDP 图像传输测试.
 * @author  x
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 UDP 图像传输功能，用于测试 UDP 图像传输的基本功能.
 ********************************************************************************/

// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
// 目标IP地址（UDP接收端）
const std::string TARGET_IP    = "192.168.45.155";
// UDP目标端口（需与上位机保持一致）
const uint16_t    TARGET_PORT  = 8080;
// 摄像头参数
const uint16_t    CAM_WIDTH    = 160;     // 宽
const uint16_t    CAM_HEIGHT   = 120;     // 高
const uint16_t    CAM_FPS      = 60;     // 帧率
// JPEG编码质量 (1-100)
const uint8_t     JPEG_QUALITY = 50;//30;

void all_void(void);


// 实车时建议由视觉线程/进程更新共享变量，电机控制线程只读取最新一帧结果。
static zmg::VisionInfo get_vision_info_from_camera(void)
{
    zmg::VisionInfo info;
    info.error = ImageStatus.Det_True-40;//0.0f;
    info.road_type = zmg::RoadType::STRAIGHT;
    info.line_lost = false;
    info.need_slow = false;
    info.need_stop = false;
    return info;
}

/********************************************************************************
 *  @brief   UDP 图像传输测试.
 *  @param   none.
 *  @return  none.
 *  @note    测试内容为 UDP 图像传输，使用OpenCV 读取摄像头图像，并使用 UDP 发送图像数据.
 *! @note    使用时需搭配对应上位机 LoongHost.exe，用于接收并显示图像.
 ********************************************************************************/
void x_test_demo(void)
{
    printf("=========================================\n");
    printf("  UDP 图传\n");
    printf("=========================================\n");
    printf("Target IP:   %s\n", TARGET_IP.c_str());
    printf("Target Port: %d\n", TARGET_PORT);
    printf("Resolution:  %dx%d\n", CAM_WIDTH, CAM_HEIGHT);
    printf("FPS:         %d\n", CAM_FPS);
    printf("=========================================\n");
    // 初始化UDP客户端
    lq_udp_client udp_client(TARGET_IP, TARGET_PORT);
    printf("UDP 客户端初始化完成\n");
    // 初始化摄像头
    lq_camera_ex cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS, LQ_CAMERA_0CPU_MJPG);
    if (!cam.is_cam_opened()) {
        printf("ERROR: 打开摄像头失败!\n");
        return;
    }
    printf("摄像头参数: %dx%d @ %dfps\n", cam.get_camera_width(), cam.get_camera_height(), cam.get_camera_fps());
    // 发送帧计数
    uint32_t frame_count = 0, encoder_count = 0;
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    printf("图传开始... 按下 Ctrl+C 停止\n");

    extern uint8 Image_Use[LCDH][LCDW];
    ImageStatus.Threshold_detach = 180;
    ImageStatus.Threshold_static = 100;
    ImageScanInterval = 3;
    ImageScanInterval_Cross = 6;

    zmg::MotorControlConfig config = zmg::default_motor_control_config();
    // 先低速小 PWM 偏移上车调试，确认 5000 停车、方向、编码器正负号和急停都正确后再提速。
    // 想改“基础速度”：直道改 straight_speed，弯道改 curve_speed；need_slow=true 时会再乘 slow_ratio。
    config.straight_speed = 1.0f;  // 直道基础速度
    config.curve_speed = 3.5f;     // 弯道基础速度
    config.max_reverse_speed = 2.0f;
    config.min_run_pwm_offset = 250;
    config.max_pwm_offset_from_mid = 1800; // duty 限制为 3200..6800，确认安全后再逐步增大。
    zmg::MotorController motor(config);

    int loop = 0;
    float test_speed = 1.0f;

    cv::Mat frame;
    cv::Mat gray;   //160*120
    cv::Mat Image_u;  //灰度80*60
    cv::Mat colorImg;
    while (ls_system_running.load()) {
        // ===================== 获取并发送图像 =====================
        // 获取原始图像
        frame = cam.get_frame_raw();
        if (frame.empty()) {
            printf("ERROR: 读取图像失败!\n");
            continue;
        }

        // ================test========
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::resize(gray, Image_u, cv::Size(80, 60), 0, 0, cv::INTER_LINEAR);
          
        // 把 Mat 数据直接复制到数组
        memcpy(Image_Use, Image_u.data, 80*60);
        ImageProcess(); //图像处理主函数
        
        // 转为三通道彩色图像用于绘制红点
        cv::cvtColor(Image_u, colorImg, cv::COLOR_GRAY2BGR);

        for (int i = 59; i > ImageStatus.OFFLine; i--)
        { 
            cv::circle(colorImg, cv::Point(ImageDeal[i].Center, i), 1, cv::Scalar(0, 0, 255), -1);
        }
        #if 0
        char recv_str[64];
        udp_client.udp_recv(recv_str, sizeof(recv_str)-1);
        if (sizeof(recv_str) > 0) {
        recv_str[sizeof(recv_str)] = '\0';  // 手动添加字符串结束符
        lq_log_info("%s", recv_str);
        } else if (sizeof(recv_str) == 0) lq_log_info("Received empty data");   // 收到空数据
        else lq_log_error("UDP receive failed"); // 接收错误
        #endif
        // ================发送JPEG压缩图像
        if (udp_client.udp_send_image(colorImg, JPEG_QUALITY) < 0) {
            printf("ERROR: 发送图像失败!\n");
        }

        //================================电机驱动
        const zmg::VisionInfo vision = get_vision_info_from_camera();

        if (loop>20 && loop%10==0 && loop<45)
        {
            test_speed += 0.5f;
        }else if (loop>=50 && loop<60 && loop%10==0)
        {
            test_speed += 4.0f;
        }else if (loop>=60 && loop<100 && loop%10==0)
        {
            test_speed -= 1.0f;
        }
        else if (loop>=100){
            motor.safe_shutdown();
            return;
        }
        loop++;
        motor.set_straight_speed(test_speed);
        lq_log_warn("test_speed:%7.2f", test_speed);
        motor.update(vision);

        const zmg::MotorControlDebug& dbg = motor.debug();
        // encL/encR 就是 dbg.left_measured_speed/right_measured_speed：
        // 左右编码器采集并换算出的速度反馈值。
        printf("base=%5.2f turn=%5.2f targetL=%5.2f targetR=%5.2f encL=%5.2f encR=%5.2f boff=%5d loff=%5d roff=%5d dutyL=%5d dutyR=%5d lost=%u\n",
            dbg.base_target_speed,
            dbg.turn_target_speed,
            dbg.left_target_speed,
            dbg.right_target_speed,
            dbg.left_measured_speed,
            dbg.right_measured_speed,
            dbg.base_pwm_offset,
            dbg.left_pwm_offset,
            dbg.right_pwm_offset,
            dbg.left_duty,
            dbg.right_duty,
            dbg.lost_line_count);
        lq_log_info("loop:%7.2f", loop);
        usleep((useconds_t)(config.control_period_s * 1000000.0f));


        char encoder_str[128];
        // 如果有两个
        float ch1 = dbg.base_pwm_offset;
        float ch2 = 0.0f;//dbg.right_measured_speed;
        float ch3 = 500.0f;
        float ch4 = dbg.left_pwm_offset;
        float ch5 = dbg.right_pwm_offset;
        snprintf(encoder_str, sizeof(encoder_str), "ch1/base:%7.2f,ch2/:%7.2f,ch3/base:%7.2f, ch4/left:%7.2f, ch5/right:%7.2f", ch1, ch2, ch3, ch4, ch5);
        // 发送编码器数据
        udp_client.udp_send_string(encoder_str);
        
        #if 0 //打印Pixle
        char lineBuffer[LCDW + 1];   // 多一个字符存放字符串结束符 '\0'
        for (int kk = 0; kk < LCDH; kk++) {
            // 1. 构建当前行的字符串
            for (int ll = 0; ll < LCDW; ll++) {
                lineBuffer[ll] = (Pixle[kk][ll] == 1) ? '*' : ' ';
            }
            lineBuffer[LCDW] = '\0';   // 字符串结束符

            // 2. 一次输出整行 + 附加的 Center 信息
            printf("%s   X_site:%3d  Y_site:%3d\n", lineBuffer, ImageDeal[kk].Center, kk);
        }
        #endif

        frame_count++;
        // ===================== 每秒打印状态 =====================
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed >= 1) {
            float fps = (float)frame_count / (float)elapsed;
            printf("FPS: %.2f\n", fps);
            frame_count = encoder_count = 0;
            start_time = now;
        }
        
    }
    // 退出前再次保持 50% duty 和 ENABLE 高，避免底层 PWM cleanup 输出危险的 0% duty。
    motor.safe_shutdown();
}

//=============================================================================
void all_void(void)
{
    lq_log_warn("666");
}
