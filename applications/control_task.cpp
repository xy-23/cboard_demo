#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;

sp::RM_Motor motor_yaw(1, sp::RM_Motors::GM6020_V, 1.0f, false);
float cmd_voltage = 0.0f;

extern "C" void control_task()
{
  while (true) {
    cmd_voltage = (remote.switch_r == sp::DBusSwitchMode::DOWN) ? 0.0f : remote.stick_lh * 24.0f;

    if (remote.switch_r == sp::DBusSwitchMode::DOWN)
      motor_yaw.cmd(0.0f);
    else
      motor_yaw.cmd(cmd_voltage);

    osDelay(1);
  }
}
