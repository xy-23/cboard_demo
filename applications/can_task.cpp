#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/lk_motor/lk_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/diff_drive/diff_drive.hpp"
#include "tools/pid/pid.hpp"

constexpr float T = 1e-3f;   // s
constexpr float R = 0.06f;   // m
constexpr float W = 0.435f;  // m

extern sp::DBus remote;

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

sp::LK_Motor motor_l(1);
sp::LK_Motor motor_r(2);

sp::PID motor_l_speed_pid(T, 1, 0, 0, 5, 5);
sp::PID motor_r_speed_pid(T, 1, 0, 0, 5, 5);

sp::DiffDrive diff_drive(W / 2, R, true, false);

extern "C" void can_task()
{
  can1.config();
  can1.start();

  can2.config();
  can2.start();

  while (true) {
    diff_drive.update(motor_l.speed, motor_r.speed);
    diff_drive.calc(remote.ch_rv * 2, -remote.ch_lh * 13);

    motor_l_speed_pid.calc(diff_drive.speed_l, motor_l.speed);
    motor_r_speed_pid.calc(diff_drive.speed_r, motor_r.speed);

    if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
      motor_l.cmd(0);
      motor_r.cmd(0);
    }
    else {
      motor_l.cmd(motor_l_speed_pid.out);
      motor_r.cmd(motor_r_speed_pid.out);
    }

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