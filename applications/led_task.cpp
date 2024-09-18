#include "cmsis_os.h"
#include "io/led/led.hpp"

extern "C" void led_task()
{
  io::Led rgb(&htim5);
  rgb.set(0, 65535, 0);

  while (true) {
    osDelay(1000);
  }
}