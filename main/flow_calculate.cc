#include "flow_calculate.h"
#include "device_config.h"
#include "feature_provider.h"

#include <stdint.h>
#include <stdio.h>

const float flow_coe[32] = {0.5};

const uint8_t lower_bound_index = 10 ;
const uint8_t upper_bound_index = 14 ;

int calculate_flow(float input_slice[]){
    int curr_flow = 0 ;
    for(int i = 0 ; i < BAND_SIZE ; i++){
        curr_flow += input_slice[i] * flow_coe[i] ;
    }
    return curr_flow;
}

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