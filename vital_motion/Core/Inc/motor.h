/**
 * @file motor.h
 * @brief M5: шаговый драйвер 4 осей (TIM1..4 + DIR PF0..PF3).
 */
#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

void motor_init(void);

/**
 * @brief Движение 4 осей одновременно.
 * @param steps[4] знаковые шаги A/B/C/D
 * @param tim_arr[4] ARR таймеров A/B/C/D (меньше = быстрее)
 * @return 0 OK, -1 timeout
 */
int motor_move_4axes(const int32_t steps[4], const uint32_t tim_arr[4]);

/** Вызывать из HAL_TIM_PeriodElapsedCallback для TIM1..TIM4. */
void motor_tim_period_elapsed(TIM_HandleTypeDef *htim);

/** Telemetry helpers. */
int32_t motor_axis_pos_steps(uint8_t axis);
uint8_t motor_axis_in_motion(uint8_t axis);
uint8_t motor_any_in_motion(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
