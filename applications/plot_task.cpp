#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

extern sp::RM_Motor motor_yaw;
extern float cmd_voltage;

extern sp::PM02 pm02;

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    // plotter.plot(pm02.power_heat_count);
    osDelay(10);
  }
}
