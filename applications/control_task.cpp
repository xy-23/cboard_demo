#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;

sp::RM_Motor motor_yaw(1, sp::RM_Motors::GM6020_V, 1.0f, false);

extern "C" void control_task()
{
  while (true) {
    if (remote.switch_r == sp::DBusSwitchMode::DOWN) {
      motor_yaw.cmd(0.0f);
    }
    else {
      motor_yaw.cmd(remote.stick_lh * 10.0f);
    }

    osDelay(1);
  }
}
