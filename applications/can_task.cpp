#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

sp::RM_Motor motor_yaw(1, sp::RM_Motors::GM6020_V, 1, false);
sp::RM_Motor motor_pitch(2, sp::RM_Motors::GM6020_V, 1, false);

extern "C" void can_task()
{
  can1.config();
  can1.start();

  can2.config();
  can2.start();

  while (true) {
    if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
      motor_yaw.cmd(0);
      motor_pitch.cmd(0);
    }
    else {
      motor_yaw.cmd(remote.ch_rh * 24);
      motor_pitch.cmd(remote.ch_rv * 24);
    }

    motor_yaw.write(can2.tx_data);
    can2.send(motor_yaw.tx_id);

    motor_pitch.write(can1.tx_data);
    can1.send(motor_pitch.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_pitch.rx_id) motor_pitch.read(can1.rx_data, stamp_ms);
    }

    else if (hcan == &hcan2) {
      can2.recv();

      if (can2.rx_id == motor_yaw.rx_id) motor_yaw.read(can2.rx_data, stamp_ms);
    }
  }
}