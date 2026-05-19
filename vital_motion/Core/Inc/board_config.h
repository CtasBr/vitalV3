/**
 * @file board_config.h
 * @brief Плата / канал связи с Mac.
 *
 * NUCLEO: один USB ST-Link → USART3 (VCP). Motion + отладочный лог на одном порту.
 * Кастомная плата манипулятора: часто USART2 (PD5/PA3) — смени на 0 и пересобери.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/** 1 = USART3 (ST-Link USB), 0 = USART2 (PD5/PA3) */
#define MOTION_LINK_USE_USART3 1

#endif /* BOARD_CONFIG_H */
