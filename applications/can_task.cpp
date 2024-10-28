#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);

extern "C" void can_task()
{
  can1.config();
  can1.start();

  while (true) {
    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  // auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1) {
    can1.recv();
  }
}