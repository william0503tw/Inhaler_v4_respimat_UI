#ifndef _AUDIO_PROVIDER_H_
#define _AUDIO_PROVIDER_H_

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <cstdint>

void I2S_begin();
void I2S_read(int32_t samples[], int sample_size);
void I2S_stop();

#endif