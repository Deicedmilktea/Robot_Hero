#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
#include "miniPC_process.h"

// extern INS_t INS;
// extern UART_HandleTypeDef huart1;

static Vision_Recv_s *vision_recv_data;

void exchange_task(void const *argument)
{
    // Vision_Init_Config_s config = {
    //     .recv_config = {
    //         .header = VISION_RECV_HEADER,
    //     },
    //     .send_config = {
    //         .header        = VISION_SEND_HEADER,
    //         .detect_color  = VISION_DETECT_COLOR_BLUE,
    //         .reset_tracker = VISION_RESET_TRACKER_NO,
    //         .is_shoot      = VISION_SHOOTING,
    //         .tail          = VISION_SEND_TAIL,
    //     },
    //     .usart_config = {
    //         .recv_buff_size = VISION_RECV_SIZE,
    //         .usart_handle   = &huart1,
    //     },
    // };
    // vision_recv_data = VisionInit(&config);

    while (1) {
        // VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);
        // VisionSend();

        osDelay(1);
    }
}