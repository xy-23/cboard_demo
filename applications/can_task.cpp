#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can2(&hcan2);

extern "C" void can_task()
{
  can2.config();
  can2.start();

  while (true) {
    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan2) {
    can2.recv();

    // if (can2.rx_id == motor_yaw.rx_id) motor_yaw.read(can2.rx_data, stamp_ms);
  }
}