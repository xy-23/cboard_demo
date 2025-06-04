#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/lk_motor/lk_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/diff_drive/diff_drive.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088;
extern sp::PM02 pm02;
extern sp::RM_Motor trigger_motor;
extern float shoot_speed;
extern float shoot_num;
extern float trigger_angle;
extern float debug_data;
extern uint8_t first_in_flag;

extern "C" void plot_task()
{
  sp::Plotter plotter(&huart1);

  while (true) {
    plotter.plot(
      pm02.robot_status.shooter_barrel_heat_limit, pm02.power_heat.shooter_17mm_1_barrel_heat,
      shoot_speed, shoot_num, trigger_motor.speed,
      (trigger_motor.angle - trigger_angle) / 2 / sp::PI * 7, debug_data, first_in_flag);
    osDelay(10);
  }
}
