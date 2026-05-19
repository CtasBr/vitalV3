/**
 * @file motor.c
 * @brief M3 — шаговый драйвер оси A через TIM1 PWM + счёт по Update IT.
 */
#include "motor.h"
#include "main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;

static osSemaphoreId_t motor_done_sem;
static volatile uint32_t motor_target_steps;
static volatile uint32_t motor_done_steps;
static volatile uint8_t motor_running;

void motor_init(void)
{
  const osSemaphoreAttr_t attr = {.name = "motorDone"};
  motor_done_sem = osSemaphoreNew(1, 0, &attr);
  motor_running = 0;
  motor_target_steps = 0;
  motor_done_steps = 0;

  HAL_GPIO_WritePin(aDir_GPIO_Port, aDir_Pin, GPIO_PIN_RESET);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim1);
}

int motor_axis_a_move(int32_t steps, uint32_t tim_arr)
{
  if (steps == 0)
  {
    return 0;
  }
  if (tim_arr < 500U)
  {
    tim_arr = 500U;
  }
  if (tim_arr > 20000U)
  {
    tim_arr = 20000U;
  }

  const uint32_t n = (steps > 0) ? (uint32_t)steps : (uint32_t)(-steps);
  const GPIO_PinState dir_pin = (steps > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(aDir_GPIO_Port, aDir_Pin, dir_pin);

  __HAL_TIM_SET_AUTORELOAD(&htim1, tim_arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tim_arr / 2U);

  motor_target_steps = n;
  motor_done_steps = 0;
  motor_running = 1;

  (void)osSemaphoreAcquire(motor_done_sem, 0);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    motor_running = 0;
    return -1;
  }
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    motor_running = 0;
    return -1;
  }

  if (osSemaphoreAcquire(motor_done_sem, 30000) != osOK)
  {
    motor_running = 0;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim1);
    return -1;
  }

  motor_running = 0;
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim1);
  return 0;
}

void motor_tim1_period_elapsed(void)
{
  if (!motor_running)
  {
    return;
  }

  motor_done_steps++;
  if (motor_done_steps >= motor_target_steps)
  {
    motor_running = 0;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim1);
    (void)osSemaphoreRelease(motor_done_sem);
  }
}
