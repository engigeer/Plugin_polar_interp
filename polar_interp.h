#ifndef _POLAR_INTERP_H_
#define _POLAR_INTERP_H_

#include <math.h>

#include "grbl/hal.h"

#define POLAR_INTERP_ENABLE_MCODE   (user_mcode_t)112
#define POLAR_INTERP_DISABLE_MCODE  (user_mcode_t)113

typedef struct {
    float pole_offset;      // work-coordinate value of radial_axis where radius = 0
    uint8_t radial_axis;    // Z_AXIS
    uint8_t rotary_axis;    // C_AXIS
} polar_settings_t;

void polar_interp_init(void);

#endif