#include "flow_calculate.h"
#include "device_config.h"
#include "feature_provider.h"

#include <stdint.h>
#include <stdio.h>

const float flow_coe[32] = {-0.12952856, -0.05259436,  0.07289376, -0.13612601, -0.02099442,  0.25322522,
  0.11442108, -0.09741415,  0.1440625,  -0.12759439, -0.03773858,  0.08530015,
  0.03278549,  0.11257013, -0.16840489, -0.140832,    0.42440075,  0.38167247,
  0.00755069 -0.00614215,  0.49709899, -0.88478113, -0.38242104,  0.56564547,
  0.49238221,  2.33506053, -0.21299519,  0.33815729, -2.9659553,   0.58249547,
  1.68404108, -0.9557001};

int calculate_flow(float input_slice[]){
    int curr_flow = 0 ;
    for(int i = 0 ; i < BAND_SIZE ; i++){
        curr_flow += input_slice[i] * flow_coe[i] ;
    }

    curr_flow = moving_average_filter_2(curr_flow);

    if(curr_flow <= 0){
        curr_flow = 0 ;
    }

    return curr_flow;
}


const uint8_t lower_bound_index = 10 ;
const uint8_t upper_bound_index = 14 ;
int calculate_flow_2(float input_slice[]){
    int curr_data ;
    int inhale_band_score = 0 ;
    for(int i = lower_bound_index ; i <= upper_bound_index ; i++){
        inhale_band_score += input_slice[i];
    }

    curr_data = moving_average_filter_2(inhale_band_score);

    //printf("inhale_band_score: %d\n", inhale_band_score);
    // if(curr_data < 300){
    //     return 30;
    // }else if(curr_data > 300 && inhale_band_score < 400){
    //     return 40;
    // }else if(curr_data > 400){
    //     return 60;
    // }else{
    //     return 0;
    // }
    return (((float)curr_data) / 650) * 60 ;
}