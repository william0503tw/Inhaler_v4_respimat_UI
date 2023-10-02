#ifndef _FEATURE_PROVIDER_H_
#define _FEATURE_PROVIDER_H_

#include "device_config.h"
#include "audio_provider.h"
#include <cstdint>

void transform_feature_one_slice(int32_t input_audio_sample[], float* band_result);
void transform_feature_one_slice_no_normalize(int32_t input_audio_sample[], float band_result[]);

void generate_feature_one_set(float dest[], float dest_std[]);


void generate_feature_one_slice(float dest[]);

void feature_provider_begin();

float moving_average_filter(float input);
float moving_average_filter_2(float input);

float find_row_max(float array[]);

#endif