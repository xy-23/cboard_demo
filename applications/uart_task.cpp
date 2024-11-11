#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"

sp::DBus remote(&huart3);
sp::PM02 pm02(&huart6);

extern "C" void uart_task()
{
  remote.request();
  pm02.request();

  while (true) {
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    remote.update(osKernelSysTick());
    remote.request();
  }
  else if (huart == &huart6) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  else if (huart == &huart6) {
    pm02.request();
  }
}