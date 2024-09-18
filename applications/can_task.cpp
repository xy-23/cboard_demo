#include "can.h"
#include "cmsis_os.h"

extern "C" void can_task()
{
  while (true) {
    osDelay(1);
  }
}