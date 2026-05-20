/**
 * @file motor.c
 * @brief M5 — шаговый драйвер 4 осей через TIM1..4 PWM + Update IT.
 */
#include "motor.h"
#include "main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  volatile uint32_t target_steps;
  volatile uint32_t done_steps;
  volatile uint8_t running;
  volatile int32_t pos_steps;
  volatile int8_t dir_sign;
} axis_rt_t;

static axis_rt_t g_axes[4];
static osSemaphoreId_t g_done_sems[4];

void motor_init(void)
{
  const osSemaphoreAttr_t attrs[4] = {
      {.name = "motorA"}, {.name = "motorB"}, {.name = "motorC"}, {.name = "motorD"}};

  g_axes[0] = (axis_rt_t){.htim = &htim1,
                          .channel = TIM_CHANNEL_1,
                          .dir_port = aDir_GPIO_Port,
                          .dir_pin = aDir_Pin,
                          .dir_sign = 1};
  g_axes[1] = (axis_rt_t){.htim = &htim2,
                          .channel = TIM_CHANNEL_1,
                          .dir_port = bDir_GPIO_Port,
                          .dir_pin = bDir_Pin,
                          .dir_sign = 1};
  g_axes[2] = (axis_rt_t){.htim = &htim3,
                          .channel = TIM_CHANNEL_1,
                          .dir_port = cDir_GPIO_Port,
                          .dir_pin = cDir_Pin,
                          .dir_sign = 1};
  g_axes[3] = (axis_rt_t){.htim = &htim4,
                          .channel = TIM_CHANNEL_1,
                          .dir_port = dDir_GPIO_Port,
                          .dir_pin = dDir_Pin,
                          .dir_sign = 1};

  for (uint8_t i = 0; i < 4; i++)
  {
    g_done_sems[i] = osSemaphoreNew(1, 0, &attrs[i]);
    g_axes[i].running = 0;
    g_axes[i].target_steps = 0;
    g_axes[i].done_steps = 0;
    g_axes[i].pos_steps = 0;
    HAL_GPIO_WritePin(g_axes[i].dir_port, g_axes[i].dir_pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(g_axes[i].htim, g_axes[i].channel);
    HAL_TIM_Base_Stop_IT(g_axes[i].htim);
  }
}

static int motor_axis_start(uint8_t axis, int32_t steps, uint32_t tim_arr)
{
  axis_rt_t *ax = &g_axes[axis];
  if (steps == 0 || ax->htim == NULL)
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
  ax->dir_sign = (steps > 0) ? 1 : -1;

  HAL_GPIO_WritePin(ax->dir_port, ax->dir_pin, dir_pin);

  __HAL_TIM_SET_AUTORELOAD(ax->htim, tim_arr);
  __HAL_TIM_SET_COMPARE(ax->htim, ax->channel, tim_arr / 2U);

  ax->target_steps = n;
  ax->done_steps = 0;
  ax->running = 1;

  (void)osSemaphoreAcquire(g_done_sems[axis], 0);

  if (HAL_TIM_PWM_Start(ax->htim, ax->channel) != HAL_OK)
  {
    ax->running = 0;
    return -1;
  }
  if (HAL_TIM_Base_Start_IT(ax->htim) != HAL_OK)
  {
    HAL_TIM_PWM_Stop(ax->htim, ax->channel);
    ax->running = 0;
    return -1;
  }
  return 0;
}

int motor_move_4axes(const int32_t steps[4], const uint32_t tim_arr[4])
{
  uint8_t used[4] = {0, 0, 0, 0};

  for (uint8_t i = 0; i < 4; i++)
  {
    if (steps[i] == 0)
    {
      continue;
    }
    used[i] = 1;
    if (motor_axis_start(i, steps[i], tim_arr[i]) != 0)
    {
      return -1;
    }
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    if (!used[i])
    {
      continue;
    }
    if (osSemaphoreAcquire(g_done_sems[i], 30000) != osOK)
    {
      g_axes[i].running = 0;
      HAL_TIM_PWM_Stop(g_axes[i].htim, g_axes[i].channel);
      HAL_TIM_Base_Stop_IT(g_axes[i].htim);
      return -1;
    }
  }
  return 0;
}

void motor_tim_period_elapsed(TIM_HandleTypeDef *htim)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    axis_rt_t *ax = &g_axes[i];
    if (ax->htim == NULL || htim->Instance != ax->htim->Instance)
    {
      continue;
    }
    if (!ax->running)
    {
      return;
    }

    ax->done_steps++;
    ax->pos_steps += (int32_t)ax->dir_sign;
    if (ax->done_steps >= ax->target_steps)
    {
      ax->running = 0;
      HAL_TIM_PWM_Stop(ax->htim, ax->channel);
      HAL_TIM_Base_Stop_IT(ax->htim);
      (void)osSemaphoreRelease(g_done_sems[i]);
    }
    return;
  }
}

int32_t motor_axis_pos_steps(uint8_t axis)
{
  if (axis >= 4)
  {
    return 0;
  }
  return g_axes[axis].pos_steps;
}

uint8_t motor_axis_in_motion(uint8_t axis)
{
  if (axis >= 4)
  {
    return 0;
  }
  return g_axes[axis].running;
}

uint8_t motor_any_in_motion(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    if (g_axes[i].running)
    {
      return 1;
    }
  }
  return 0;
}

void motor_estop_all(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    axis_rt_t *ax = &g_axes[i];
    ax->running = 0;
    HAL_TIM_PWM_Stop(ax->htim, ax->channel);
    HAL_TIM_Base_Stop_IT(ax->htim);
    (void)osSemaphoreRelease(g_done_sems[i]);
  }
}
