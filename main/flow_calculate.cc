#include "flow_calculate.h"
#include "device_config.h"

const float flow_coe[32] = {0.5};

int calculate_flow(float input_slice[]){
    int curr_flow = 0 ;
    for(int i = 0 ; i < BAND_SIZE ; i++){
        curr_flow += input_slice[i] * flow_coe[i] ;
    }
    return curr_flow;
}