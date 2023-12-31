/*
*************Chassis_task底盘任务**************
采用3508，CAN2，ID = 1234
遥控器控制：左拨杆上下→前后
           左拨杆左右→左右
           左滑轮→旋转
*/

#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
motor_info_t motor_can2[6]; // can2电机信息结构体, 0123：底盘，4：拨盘, 5: 云台
chassis_t chassis[4];
volatile int16_t Vx = 0, Vy = 0, Wz = 0;
int16_t Temp_Vx;
int16_t Temp_Vy;
int fllowflag = 0;
int16_t relative_yaw = 0;
extern RC_ctrl_t rc_ctrl;
extern INS_t INS;
extern INS_t INS_top;
extern uint16_t shift_flag;

int error10 = 0;
fp32 speed10 = 0;

double rx = 0.2, ry = 0.2;
// Save imu data

int16_t chassis_mode = 1; // 判断底盘状态，用于UI编写
int16_t chassis_speed_max = 2000;

int chassis_mode_flag = 0;

void Chassis_task(void const *pvParameters)
{

  Chassis_loop_Init();

  for (;;)
  {
    Calculate_speed();
    chassis_current_give();
    error10++;
    osDelay(1);
  }
}

static void Chassis_loop_Init()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis[i].pid_value[0] = 30;
    chassis[i].pid_value[1] = 0.5;
    chassis[i].pid_value[2] = 0;
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis[i].pid, chassis[i].pid_value, 6000, 6000);
  }

  Vx = 0;
  Vy = 0;
  Wz = 0;
}

// speed mapping
int16_t Speedmapping(int value, int from_min, int from_max, int to_min, int to_max)
{
  // 首先将输入值从 [a, b] 映射到 [0, 1] 范围内
  double normalized_value = (value * 1.0 - from_min) / (from_max - from_min);

  // 然后将标准化后的值映射到 [C, D] 范围内
  int16_t mapped_value = (int16_t)(to_min + (to_max - to_min) * normalized_value);

  return mapped_value;
}

void Calculate_speed()
{
  Vx = Speedmapping(rc_ctrl.rc.ch[2], -660, 660, -chassis_speed_max, chassis_speed_max); // left and right
  Vy = Speedmapping(rc_ctrl.rc.ch[3], -660, 660, -chassis_speed_max, chassis_speed_max); // front and back
  Wz = Speedmapping(rc_ctrl.rc.ch[4], -660, 660, -chassis_speed_max, chassis_speed_max); // rotate

  int16_t Temp_Vx = Vx;
  int16_t Temp_Vy = Vy;

  // int16_t relative_yaw = 0;
  relative_yaw = INS.Yaw - INS_top.Yaw;
  relative_yaw = -relative_yaw / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反

  Vx = cos(relative_yaw) * Temp_Vx - sin(relative_yaw) * Temp_Vy;
  Vy = sin(relative_yaw) * Temp_Vx + cos(relative_yaw) * Temp_Vy;

  chassis[0].target_speed = Vy + Vx + 3 * Wz * (rx + ry);
  chassis[1].target_speed = -Vy + Vx + 3 * Wz * (rx + ry);
  chassis[2].target_speed = -Vy - Vx + 3 * Wz * (rx + ry);
  chassis[3].target_speed = Vy - Vx + 3 * Wz * (rx + ry);
}

// 电机电流控制
void chassis_current_give()
{

  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    motor_can2[i].set_current = pid_calc(&chassis[i].pid, chassis[i].target_speed, motor_can2[i].rotor_speed);
  }

  chassis_can2_cmd(motor_can2[0].set_current, motor_can2[1].set_current, motor_can2[2].set_current, motor_can2[3].set_current);
}

// CAN2发送信号
void chassis_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  tx_header.StdId = 0x200;
  tx_header.IDE = CAN_ID_STD;   // 标准帧
  tx_header.RTR = CAN_RTR_DATA; // 数据帧
  tx_header.DLC = 8;            // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] = (v3) & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] = (v4) & 0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}
