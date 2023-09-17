#ifndef _MODEL_SETTINGS_H_
#define _MODEL_SETTINGS_H_

typedef enum{
  NOISE_INDEX = 0,
  INHALE_INDEX
} inhaler_prediction_index_t ;

constexpr int kFeatureSliceSize = 32;
constexpr int kFeatureSliceCount = 32;
constexpr int kFeatureElementCount = (kFeatureSliceSize * kFeatureSliceCount);

constexpr int kCategoryCount = 2;
extern const char* kCategoryLabels[kCategoryCount];



#endif