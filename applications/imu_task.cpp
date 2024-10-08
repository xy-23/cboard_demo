#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"

io::Plotter plotter(&huart1);
io::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0);

int plot_cnt = 0;

extern "C" void imu_task()
{
  osDelay(100);
  bmi088.init();

  while (true) {
    bmi088.update();
    osDelay(1);

    if (plot_cnt == 10) {
      plot_cnt = 0;
      plotter.plot(
        bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1],
        bmi088.gyro[2]);
    }
    plot_cnt++;
  }
}