#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

extern void Motor_ControlLoop_20ms(void);
static uint32_t systick_20ms_counter = 0;

void SysTick_Handler(void)
{
  HAL_IncTick();
  systick_20ms_counter++;
  if (systick_20ms_counter >= 20)
  {
    systick_20ms_counter = 0;
    Motor_ControlLoop_20ms();
  }
}

void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}
