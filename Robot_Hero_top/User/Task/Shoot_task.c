/*
**********Shoot_task射击任务**********
包含对两个摩擦轮的控制
摩擦轮分别为3508，ID = 5（left）和 ID = 6（right），CAN2 控制
遥控器右边拨杆控制，拨到最上面启动 (从上到下分别为132)
*/

#include "Shoot_task.h"
#include "pid.h"
#include "cmsis_os.h"
#include "main.h"
#include "rc_potocal.h"

shoot_t shoot_motor[2];            // 摩擦轮can2，id = 56
motor_info_t motor_can2[3];        //[2]:pitch
int16_t friction_max_speed = 8000; // 摩擦轮速度
extern RC_ctrl_t rc_ctrl;

void Shoot_task(void const *argument)
{
  shoot_loop_init();

  for (;;)
  {
    // 遥控器右边拨到上和中，电机启动
    if (rc_ctrl.rc.s[0] == 1 || rc_ctrl.rc.s[0] == 3)
    {
      shoot_start();
    }
    else
    {
      shoot_stop();
    }

    // shoot_stop();

    // //遥控器左边拨到下，弹仓盖打开
    // if(rc_ctrl.rc.s[1] == 2){
    //   shoot_lid_open();
    // }
    // else{
    //   shoot_lid_close();
    // }

    shoot_current_give();
    osDelay(1);
  }
}

/***************初始化***************/
void shoot_loop_init()
{
  // friction_left
  shoot_motor[0].pid_value[0] = 20;
  shoot_motor[0].pid_value[1] = 5;
  shoot_motor[0].pid_value[2] = 1;

  // friction_right
  shoot_motor[1].pid_value[0] = 20;
  shoot_motor[1].pid_value[1] = 5;
  shoot_motor[1].pid_value[2] = 1;

  // 初始化目标速度
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;

  // 初始化PID
  pid_init(&shoot_motor[0].pid, shoot_motor[0].pid_value, 1000, friction_max_speed); // friction_right
  pid_init(&shoot_motor[1].pid, shoot_motor[1].pid_value, 1000, friction_max_speed); // friction_left
}

/***************射击模式*****************/
void shoot_start()
{
  shoot_motor[0].target_speed = 5000;
  shoot_motor[1].target_speed = 5050;
}

/***************停止射击模式**************/
void shoot_stop()
{
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
}

/********************************摩擦轮can2发送电流***************************/
void shoot_can2_cmd(int16_t v1, int16_t v2)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = 0x1FF;
  tx_header.IDE = CAN_ID_STD;   // 标准帧
  tx_header.RTR = CAN_RTR_DATA; // 数据帧

  tx_header.DLC = 8; // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff;
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = NULL;
  tx_data[5] = NULL;
  tx_data[6] = NULL;
  tx_data[7] = NULL;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/********************************PID计算速度并发送电流****************************/
void shoot_current_give()
{

  motor_can2[0].set_current = pid_calc(&shoot_motor[0].pid, shoot_motor[0].target_speed, motor_can2[0].rotor_speed);
  motor_can2[1].set_current = pid_calc(&shoot_motor[1].pid, shoot_motor[1].target_speed, -motor_can2[1].rotor_speed);

  shoot_can2_cmd(motor_can2[0].set_current, -motor_can2[1].set_current);
}
