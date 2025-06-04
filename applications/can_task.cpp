#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

constexpr float T_CONTROL = 1e-3f;    // 控制周期, 单位: s
constexpr float FRIC_SPEED = 600.0f;  //21.5-21.8

extern sp::DBus remote;

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);

sp::RM_Motor fric_motor_l(1, sp::RM_Motors::M3508, 1);
sp::RM_Motor fric_motor_r(2, sp::RM_Motors::M3508, 1);
sp::RM_Motor trigger_motor(3, sp::RM_Motors::M2006, sp::M2006_P36);

sp::PID fric_motor_l_pid(T_CONTROL, 0.0025f, 0.0000f, 0.000005f, 0.4f, 0.1f, 0.8f);
sp::PID fric_motor_r_pid(T_CONTROL, 0.0025f, 0.0000f, 0.000005f, 0.4f, 0.1f, 0.8f);

sp::PID trigger_speed_pid(T_CONTROL, 0.6f, 0.15f, 0.001f, 2.0f, 0.37f, 0.25f, false);

float trigger_target_speed = 10.0f;

extern "C" void can_task()
{
  can1.config();
  can1.start();

  can2.config();
  can2.start();

  while (true) {
    if (remote.sw_r == sp::DBusSwitchMode::UP) {
      trigger_speed_pid.calc(trigger_target_speed, trigger_motor.speed);
      trigger_motor.cmd(trigger_speed_pid.out);
    }
    else {
      trigger_motor.cmd(0);
    }

    fric_motor_l_pid.calc(
      remote.sw_r == sp::DBusSwitchMode::DOWN ? 0 : -FRIC_SPEED, fric_motor_l.speed);
    fric_motor_r_pid.calc(
      remote.sw_r == sp::DBusSwitchMode::DOWN ? 0 : FRIC_SPEED, fric_motor_r.speed);

    fric_motor_l.cmd(fric_motor_l_pid.out);
    fric_motor_r.cmd(fric_motor_r_pid.out);

    fric_motor_l.write(can1.tx_data);
    fric_motor_r.write(can1.tx_data);
    trigger_motor.write(can1.tx_data);
    can1.send(trigger_motor.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == fric_motor_l.rx_id) {
        fric_motor_l.read(can1.rx_data, stamp_ms);
      }
      else if (can1.rx_id == fric_motor_r.rx_id) {
        fric_motor_r.read(can1.rx_data, stamp_ms);
      }
      else if (can1.rx_id == trigger_motor.rx_id) {
        trigger_motor.read(can1.rx_data, stamp_ms);
      }
    }

    else if (hcan == &hcan2) {
      can2.recv();
    }
  }
}