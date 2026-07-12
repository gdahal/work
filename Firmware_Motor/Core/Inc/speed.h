#ifndef __SPEED_H
#define __SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Configurable constants in header files
#ifndef MOTOR_POLE_PAIRS
#define MOTOR_POLE_PAIRS            8    // Number of rotor pole pairs (16 magnets)
#endif

#ifndef HALL_EDGES_PER_ELEC_REV
#define HALL_EDGES_PER_ELEC_REV     6    // 6 Hall transitions per electrical revolution
#endif

#define FILTER_SAMPLES              8    // Size of moving average filter

void Speed_Init(void);
void Speed_Reset(void);
void Speed_ProcessTransition(uint32_t current_ticks);
uint16_t Speed_GetRPM(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_H */
