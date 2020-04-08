#include "static-conf.h"

// Offsets de bateria para deimos (battMode=2)
const float VBAT_aux = 0.3197278911564626;
const float VIN_aux = 0.0448901623686724;
const float VSYS_aux = 0.3197278911564626;

// Offsets de bateria para halley (battMode=3)
const float VBAT_aux_HALLEY = 0.3197278911564626;
const float VIN_aux_HALLEY = 0.0231513138614829;
const float VSYS_aux_HALLEY = 0.3197278911564626;

float mV_step_used = 0.00322265625; // VREF 3v3

#if RH_battMode == 3
#include "TLA2024.h"
TLA2024 adc = TLA2024();
unsigned char channel = AIN0;
#endif