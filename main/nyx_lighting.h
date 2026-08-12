#ifndef MAIN_NYX_LIGHTING_H_
#define MAIN_NYX_LIGHTING_H_

#include <stdbool.h>
#include <stdint.h>

/* Starts the firmware-owned WS2812 renderer. */
void nyx_lighting_init(int pin, unsigned int timing_preset);

/* Applies Lighting/Brake values after the main Express configuration changes. */
void nyx_lighting_config_changed(void);

#endif /* MAIN_NYX_LIGHTING_H_ */
