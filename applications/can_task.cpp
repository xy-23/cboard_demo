#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

io::CAN can1(&hcan1);
motor::GM6020 motor6020(1, false);  // 一个电机id为1, 电流控制模式的6020

extern "C" void can_task()
{
  can1.config();
  can1.start();

  while (true) {
    motor6020.cmd(0.0f);  // 调整为0.1f, 电机会旋转, 注意安全

    motor6020.write(can1.tx_data);
    can1.send(motor6020.tx_id());

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1) {
    can1.recv();

    if (can1.rx_id == motor6020.rx_id()) motor6020.read(can1.rx_data, stamp_ms);
  }
}