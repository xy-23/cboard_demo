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
extern sp::Mahony imu;

extern sp::PM02 pm02;

extern sp::PID motor_l_speed_pid;
extern sp::PID motor_r_speed_pid;

extern sp::LK_Motor motor_l;
extern sp::LK_Motor motor_r;

extern sp::DiffDrive diff_drive;

sp::Plotter plotter(&huart1);

float power_computed = 0;

extern "C" void plot_task()
{
  while (true) {
    // plotter.plot(motor_l_speed_pid.data.set, motor_l_speed_pid.data.fdb, motor_l_speed_pid.out);
    // plotter.plot(motor_r_speed_pid.data.set, motor_r_speed_pid.data.fdb, motor_r_speed_pid.out);

    power_computed = motor_l.torque * motor_l.speed + motor_r.torque * motor_r.speed;

    plotter.plot(pm02.power_heat.reserved_3, power_computed, diff_drive.v, diff_drive.w);
    osDelay(10);
  }
}
