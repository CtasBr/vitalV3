/**
 * @file motion_limits.h
 * @brief Auto-generated from config/robot.yaml by tools/generate_motion_limits_header.py
 *        Do not edit manually.
 */
#ifndef MOTION_LIMITS_H
#define MOTION_LIMITS_H

#include <stdint.h>

#define MOTION_LIMIT_AXES 4U

static const int32_t MOTION_SOFT_LIMIT_MIN[MOTION_LIMIT_AXES] = {-4800, -4800, -4800, -4800};
static const int32_t MOTION_SOFT_LIMIT_MAX[MOTION_LIMIT_AXES] = {4800, 4800, 4800, 4800};

#endif /* MOTION_LIMITS_H */
