#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/dm_motor/dm_motor.hpp"

extern sp::DBus remote;

sp::CAN can1(&hcan1);
sp::DM_Motor motor_l(0x07, 0x17, 12.5, 20, 150);
sp::DM_Motor motor_r(0x08, 0x18, 12.5, 20, 150);

extern "C" void can_task()
{
  can1.config();
  can1.start();

  osDelay(1000);
  motor_l.write_enable(can1.tx_data);
  can1.send(motor_l.tx_id);
  motor_r.write_enable(can1.tx_data);
  can1.send(motor_r.tx_id);

  while (true) {
    if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
      motor_l.cmd(0);
      motor_r.cmd(0);
    }
    else {
      motor_l.cmd(remote.sw_l == sp::DBusSwitchMode::MID ? -80 : 0);
      motor_r.cmd(remote.sw_l == sp::DBusSwitchMode::MID ? 80 : 0);
    }

    motor_l.write(can1.tx_data);
    can1.send(motor_l.tx_id);
    motor_r.write(can1.tx_data);
    can1.send(motor_r.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  if (hcan == &hcan1) {
    can1.recv();

    if (can1.rx_id == motor_l.rx_id)
      motor_l.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motor_r.rx_id)
      motor_r.read(can1.rx_data, stamp_ms);
  }
}