#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"
#include "referee/ui/ui.hpp"

sp::DBus remote(&huart3);
sp::PM02 pm02(&huart6);

sp::UI_Manager ui_manager;

using namespace sp::ui;
Line line(Layer::LAYER_0, Color::RED_BLUE, 1, 0, 0, 100, 100);
Rectangle rect(Layer::LAYER_1, Color::YELLOW, 2, 100, 100, 200, 200);
Circle circle(Layer::LAYER_2, Color::GREEN, 3, 100, 100, 50);
Ellipse ellipse(Layer::LAYER_3, Color::ORANGE, 4, 100, 100, 50, 25);
Arc arc(Layer::LAYER_4, Color::PURPLE, 5, 200, 200, 0, 180, 25, 50);
Float float_num(Layer::LAYER_5, Color::PINK, 6, 300, 300, 60, 3.14);
Integer int_num(Layer::LAYER_6, Color::CYAN, 7, 300, 400, 70, 42);
String str(Layer::LAYER_7, Color::BLACK, 8, 300, 500, 80, "Hello, World!");

extern "C" void uart_task()
{
  remote.request();
  pm02.request();

  while (true) {
    ui_manager.set_sender_id(pm02.robot_status.robot_id);  // e.g. sp::referee::robot_id::RED_HERO

    ui_manager.pack(&str);
    pm02.send(ui_manager.data(), ui_manager.size());
    osDelay(33);

    ui_manager.pack(&line, &rect, &circle, &ellipse, &arc, &float_num, &int_num);
    pm02.send(ui_manager.data(), ui_manager.size());
    osDelay(33);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }

  else if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }

  else if (huart == pm02.huart) {
    pm02.request();
  }
}