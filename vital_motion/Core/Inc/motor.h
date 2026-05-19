/**
 * @file motor.h
 * @brief M3: ось A (TIM1 / PE9 STEP, PF0 DIR).
 */
#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void motor_init(void);

/**
 * @brief Шаги оси A (знак = направление).
 * @param steps  >0 / <0
 * @param tim_arr  период TIM1 (ARR), больше = медленнее; типично 3000–8000
 * @return 0 OK, -1 timeout
 */
int motor_axis_a_move(int32_t steps, uint32_t tim_arr);

/** Вызывать из HAL_TIM_PeriodElapsedCallback при TIM1. */
void motor_tim1_period_elapsed(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
