#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"

io::BMI088 imu(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0);

uint8_t imu_error_code;

extern "C" void imu_task()
{
  // do {
  //   osDelay(100);
  // } while (imu_error_code != 0);
  osDelay(500);
  imu_error_code = imu.init_acc();

  if (imu_error_code != 0x00)
    while (true);

  while (true) {
    imu.update();
    osDelay(10);
  }
}