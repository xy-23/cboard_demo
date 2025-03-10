#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/lk_motor/lk_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

sp::LK_Motor motor_l(1);
sp::LK_Motor motor_r(2);

extern "C" void can_task()
{
  can1.config();
  can1.start();

  can2.config();
  can2.start();

  while (true) {
    motor_l.cmd(remote.ch_lv);
    motor_r.cmd(-remote.ch_rv);

    motor_l.write(can1.tx_data);
    motor_r.write(can1.tx_data);
    can1.send(motor_l.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  // auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_l.rx_id)
        motor_l.read(can1.rx_data);

      else if (can1.rx_id == motor_r.rx_id)
        motor_r.read(can1.rx_data);
    }

    else if (hcan == &hcan2) {
      can2.recv();
    }
  }
}