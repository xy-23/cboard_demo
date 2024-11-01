#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;
extern sp::RM_Motor motor_yaw;
extern float cmd_voltage;

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(cmd_voltage, motor_yaw.angle, motor_yaw.speed, motor_yaw.torque);
    osDelay(10);
  }
}
