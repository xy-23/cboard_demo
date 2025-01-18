#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;
extern sp::DM_Motor motor_l;
extern sp::DM_Motor motor_r;

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(
      motor_l.speed, motor_r.speed, motor_l.error, -motor_r.error, motor_l.torque, motor_r.torque);
    osDelay(10);
  }
}
