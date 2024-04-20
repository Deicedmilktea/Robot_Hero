/*
*************pitch轴任务**************
采用3508，ID = 7，CAN2，motor_can2[2]
遥控器控制：左遥杆上下
*/

// #include "Pitch_task.h"
// #include "cmsis_os.h"
// #include "rc_potocal.h"
// #include "ins_task.h"

// extern motor_info_t motor_can2[5];
// extern RC_ctrl_t rc_ctrl;
// pitch_t pitch;
// float relative_pitch = 0;
// extern INS_t INS_bottom;
// extern float vision_pitch;

// // 初始化
// static void pitch_loop_init();

// /*can1发送电流*/
// static void pitch_can2_cmd(int16_t v3);

// // PID计算速度并发送电流
// static void pitch_current_give();

// // 判断pitch位置
// static void pitch_position_limit();

// void Pitch_task(void const *argument)
// {
//     pitch_loop_init();

//     for (;;)
//     {
//         relative_pitch = INS.Roll - INS_bottom.Roll;

//         // 视觉识别，右拨杆上/鼠标右键
//         if (rc_ctrl.rc.s[0] == 1 || press_right == 1)
//         {
//             // 视觉模式下的遥控器微调
//             pitch.vision_remote_pitch += (rc_ctrl.rc.ch[3] / 660.0f - rc_ctrl.mouse.y / 16384.0f * 50) * 0.1f;
//             pitch.vision_target_pitch = pitch.vision_remote_pitch + vision_pitch;

//             if (pitch.vision_target_pitch > 20)
//             {
//                 pitch.vision_target_pitch = 20;
//             }
//             if (pitch.vision_target_pitch < -15)
//             {
//                 pitch.vision_target_pitch = -15;
//             }

//             pitch.target_speed = -pid_calc(&pitch.vision_pid_angle, pitch.vision_target_pitch, INS.Roll);

//             // target_speed 的计算必须加上负号（想要符合给正值抬头，负值低头的话），与3508的旋转方向相关，否则pitch会疯转
//         }

//         else
//         {
//             pitch.target_speed = -(rc_ctrl.rc.ch[3] / 660.0f * pitch.speed_max - 200 * rc_ctrl.mouse.y / 16384.0f * pitch.speed_max);
//             pitch_position_limit();
//         }

//         pitch_current_give();
//         osDelay(1);
//     }
// }

// /****************初始化****************/
// static void pitch_loop_init()
// {
//     pitch.speed_pid_value[0] = 3;
//     pitch.speed_pid_value[1] = 0;
//     pitch.speed_pid_value[2] = 0;

//     pitch.angle_pid_value[0] = 3;
//     pitch.angle_pid_value[1] = 0;
//     pitch.angle_pid_value[2] = 0;

//     pitch.vision_speed_pid_value[0] = 20;
//     pitch.vision_speed_pid_value[1] = 0;
//     pitch.vision_speed_pid_value[2] = 0;

//     pitch.vision_angle_pid_value[0] = 400;
//     pitch.vision_angle_pid_value[1] = 0;
//     pitch.vision_angle_pid_value[2] = 0;

//     pitch.target_speed = 0;
//     pitch.speed_max = 4000;

//     pid_init(&pitch.pid_speed, pitch.speed_pid_value, 1000, 4000);
//     pid_init(&pitch.pid_angle, pitch.angle_pid_value, 1000, 4000);
//     pid_init(&pitch.vision_pid_speed, pitch.vision_speed_pid_value, 1000, 4000);
//     pid_init(&pitch.vision_pid_angle, pitch.vision_angle_pid_value, 1000, 4000);
// }

// /********************************can1发送电流***************************/
// static void pitch_can2_cmd(int16_t v3)
// {
//     uint32_t send_mail_box;
//     CAN_TxHeaderTypeDef tx_header;
//     uint8_t tx_data[8];

//     tx_header.StdId = 0x1FF;
//     tx_header.IDE = CAN_ID_STD;   // 标准帧
//     tx_header.RTR = CAN_RTR_DATA; // 数据帧

//     tx_header.DLC = 8; // 发送数据长度（字节）

//     tx_data[0] = NULL;
//     tx_data[1] = NULL;
//     tx_data[2] = NULL;
//     tx_data[3] = NULL;
//     tx_data[4] = (v3 >> 8) & 0xff;
//     tx_data[5] = (v3) & 0xff;
//     tx_data[6] = NULL;
//     tx_data[7] = NULL;

//     HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
// }

// /****************PID计算速度并发送电流***************/
// static void pitch_current_give()
// {
//     if (rc_ctrl.rc.s[1] == 2)
//     {
//         motor_can2[2].set_current = pid_calc(&pitch.vision_pid_speed, pitch.target_speed, motor_can2[2].rotor_speed);
//     }

//     else
//     {
//         motor_can2[2].set_current = pid_calc(&pitch.pid_speed, pitch.target_speed, motor_can2[2].rotor_speed);
//     }

//     pitch_can2_cmd(motor_can2[2].set_current);
// }

// /***************判断pitch位置******************/
// static void pitch_position_limit()
// {
//     if (relative_pitch > 20 && pitch.target_speed < 0) // pitch.target_speed正负与3508旋转方向有关
//     {
//         pitch.target_speed = 0;
//     }
//     if (relative_pitch < -15 && pitch.target_speed > 0)
//     {
//         pitch.target_speed = 0;
//     }
// }

/************************ 以下为发射测试拨盘代码 ***************************************/
#include "Pitch_task.h"
#include "pid.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include <stdbool.h>

#define TRIGGER_SINGLE_ANGLE 1140 // 19*360/6
#define TRIGGER_SPEED 250

trigger_t trigger; // 拨盘can1，id = 5

bool is_angle_control = false;
float current_time = 0;
float last_time = 0;

extern RC_ctrl_t rc_ctrl;
extern motor_info_t motor_can2[5];

// 初始化
static void shoot_loop_init();

// 射击模式
static void shoot_start();

// 反转
static void shoot_reverse();

// 停止射击模式
static void shoot_stop();

// 拨盘旋转固定角度
static void trigger_single_angle_move();

// 拨盘can1发送电流
static void trigger_can2_cmd(int16_t v1);

// PID计算速度并发送电流
static void shoot_current_give();

void Pitch_task(void const *argument)
{
    shoot_loop_init();

    for (;;)
    {
        // 右拨杆下，遥控器控制
        // 左拨杆上，电机启动
        if (rc_ctrl.rc.s[1] == 1)
        {
            is_angle_control = false;
            shoot_start();
        }
        else
        {
            shoot_stop();
        }

        shoot_current_give();
        osDelay(1);
    }
}

/***************初始化***************/
static void shoot_loop_init()
{
    // trigger
    // trigger.pid_value[0] = 50;
    // trigger.pid_value[1] = 1;
    // trigger.pid_value[2] = 0.05;

    trigger.pid_speed_value[0] = 30;
    trigger.pid_speed_value[1] = 0.1;
    trigger.pid_speed_value[2] = 0;

    trigger.pid_angle_value[0] = 10;
    trigger.pid_angle_value[1] = 0.05;
    trigger.pid_angle_value[2] = 500;

    // 初始化目标速度
    trigger.target_speed = 0;
    // trigger.target_angle = motor_can2[2].total_angle;

    // 初始化PID
    pid_init(&trigger.pid_speed, trigger.pid_speed_value, 20000, 30000); // trigger_speed
    pid_init(&trigger.pid_angle, trigger.pid_angle_value, 20000, 30000); // trigger_angle
}

/***************射击模式*****************/
static void shoot_start()
{
    trigger.target_speed = -TRIGGER_SPEED;
}

// /*************拨盘旋转固定角度***********/
// static void trigger_single_angle_move()
// {
//     current_time = DWT_GetTimeline_ms();
//     // 判断两次发射时间间隔，避免双发
//     if (current_time - last_time > 1000)
//     {
//         last_time = DWT_GetTimeline_ms();
//         trigger.target_angle = motor_can2[2].total_angle - TRIGGER_SINGLE_ANGLE;
//     }
// }

/*****************反转******************/
static void shoot_reverse()
{
    trigger.target_speed = TRIGGER_SPEED;
}

/***************停止射击模式**************/
static void shoot_stop()
{
    if (!is_angle_control)
        trigger.target_speed = 0;
}

/********************************拨盘can1发送电流***************************/
static void trigger_can2_cmd(int16_t v3)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = 0x1FF;
    tx_header.IDE = CAN_ID_STD;   // 标准帧
    tx_header.RTR = CAN_RTR_DATA; // 数据帧

    tx_header.DLC = 8; // 发送数据长度（字节）

    tx_data[0] = NULL;
    tx_data[1] = NULL;
    tx_data[2] = NULL;
    tx_data[3] = NULL;
    tx_data[4] = (v3 >> 8) & 0xff;
    tx_data[5] = (v3) & 0xff;
    tx_data[6] = NULL;
    tx_data[7] = NULL;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/********************************PID计算速度并发送电流****************************/
static void shoot_current_give()
{
    if (is_angle_control)
        return;
    // motor_can2[2].set_current = pid_calc_trigger(&trigger.pid_angle, trigger.target_angle, motor_can2[2].total_angle);
    else
        motor_can2[2].set_current = pid_calc(&trigger.pid_speed, trigger.target_speed, motor_can2[2].rotor_speed);

    trigger_can2_cmd(motor_can2[2].set_current);
}