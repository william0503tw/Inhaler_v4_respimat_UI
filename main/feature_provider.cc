#include "feature_provider.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdint>
#include "esp_system.h"
#include <math.h>

#include "esp_dsp.h"

const char TAG[] = "feature provider" ;


bool is_fetch_sound = 0 ;

// Window coefficients
float _wind[SAMPLE_SIZE];

// working complex array
float result[SAMPLE_SIZE*2];

float* FFT_result = &result[0];

void _findMinMax(float* array, int size, float* min, float* max) {
    *min = *max = array[0];
    for (int i = 1; i < size; ++i) {
        if (array[i] < *min) *min = array[i];
        if (array[i] > *max) *max = array[i];
    }
}

void _normalize(float* array, int size) {
    float min, max;
    _findMinMax(array, size, &min, &max);
    for (int i = 0; i < size; ++i) {
        array[i] = (array[i] - min) / (max - min) * 255;
    }
}

void _dc_remove(int32_t input[], int size){
    int64_t mean = 0 ;
    for(int i = 0 ; i < size ; i++){
        mean += input[i] ;
    }
    mean /= size ;

    for(int i = 0 ; i < size ; i++){
        input[i] -= mean ;
    }
}

void feature_provider_begin(){
    esp_err_t ret;
    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret  != ESP_OK){
        ESP_LOGE("feature_provider_begin", "Not possible to initialize FFT. Error = %i", ret);
        return;
    }else{
        ESP_LOGI("feature_provider_begin", "Start Feature Provider");
    }
}

void _transform_feature_one_slice(int32_t input_audio_sample[], float band_result[]){
    _dc_remove(input_audio_sample, SAMPLE_SIZE);

    for(int i = 0 ; i < SAMPLE_SIZE ; i++){
        result[i*2+0] =  static_cast<float>(input_audio_sample[i]); // Real
        result[i*2+1] = 0 ; // Img
    }

    // FFT
    dsps_fft2r_fc32_ae32(result, SAMPLE_SIZE);

    // Bit reverse 
    dsps_bit_rev_fc32(result, SAMPLE_SIZE);

    // Separate
    dsps_cplx2reC_fc32(result, SAMPLE_SIZE);

    for(int i = 0; i < (SAMPLE_SIZE / 2); i++) {
        FFT_result[i] =  sqrtf((FFT_result[i * 2 + 0] * FFT_result[i * 2 + 0] + FFT_result[i * 2 + 1] * FFT_result[i * 2 + 1]));
        //printf("%d: %.2f\n",i, FFT_result[i]);
    }  


    double temp = 0 ;
    int count = 0 ;
    temp += FFT_result[0];
    for(int i = 1 ; i < (SAMPLE_SIZE >> 1) ; i++){
        if(i % bin_perBand != 0){
            //printf("%lf\n", FFT_result[i]);
            temp += FFT_result[i];
        }else{
            band_result[count] = temp / bin_perBand ; 
            temp = 0 ;
            count++ ;
            if(i != 0) temp += FFT_result[i];
        }
    }
    band_result[count] = temp / bin_perBand ; 

    _normalize(band_result, BAND_SIZE);
}

void _transform_feature_one_slice_no_normalize(int32_t input_audio_sample[], float band_result[]){
    _dc_remove(input_audio_sample, SAMPLE_SIZE);

    for(int i = 0 ; i < SAMPLE_SIZE ; i++){
        result[i*2+0] =  static_cast<float>(input_audio_sample[i]); // Real
        result[i*2+1] = 0 ; // Img
    }

    // FFT
    dsps_fft2r_fc32_ae32(result, SAMPLE_SIZE);

    // Bit reverse 
    dsps_bit_rev_fc32(result, SAMPLE_SIZE);

    // Separate
    dsps_cplx2reC_fc32(result, SAMPLE_SIZE);

    for(int i = 0; i < (SAMPLE_SIZE / 2); i++) {
        FFT_result[i] =  sqrtf((FFT_result[i * 2 + 0] * FFT_result[i * 2 + 0] + FFT_result[i * 2 + 1] * FFT_result[i * 2 + 1]));
        //printf("%d: %.2f\n",i, FFT_result[i]);
    }  


    double temp = 0 ;
    int count = 0 ;
    temp += FFT_result[0];
    for(int i = 1 ; i < (SAMPLE_SIZE >> 1) ; i++){
        if(i % bin_perBand != 0){
            //printf("%lf\n", FFT_result[i]);
            temp += FFT_result[i];
        }else{
            band_result[count] = temp / bin_perBand / 1000 ; 
            temp = 0 ;
            count++ ;
            if(i != 0) temp += FFT_result[i];
        }
    }
    band_result[count] = temp / bin_perBand / 1000 ; 
}


//----------- interface -----------

void generate_feature_one_set(float dest[], float dest_std[]){
    static float result_buffer_normalize[BAND_SIZE];
    static float result_buffer_std[BAND_SIZE];
    static int32_t samples[SAMPLE_SIZE];
    int row = 0 ;
    while(row < ROW_COUNT){
        
        I2S_read(samples, SAMPLE_SIZE);
        _transform_feature_one_slice(samples, result_buffer_normalize);
        _transform_feature_one_slice_no_normalize(samples, result_buffer_std);

        for(int i = 0 ; i < BAND_SIZE ; i++){
            dest[row * BAND_SIZE + i] = result_buffer_normalize[i] ;
        }

        float temp = 0 ;
        for(int i = 0 ; i < BAND_SIZE ; i++){
            temp += result_buffer_std[i] ;
        }
        temp /= BAND_SIZE ;
        dest_std[row] = temp ;
        row++;
    }
}


void generate_feature_one_slice(float dest[]){
    static int32_t samples[SAMPLE_SIZE];
    I2S_read(samples, SAMPLE_SIZE);
    _transform_feature_one_slice(samples, dest);
}

float moving_average_filter(float input) {
    static float window[SMA_WINDOW_SIZE] = {0};
    static int index = 0;
    static double sum = 0;
    float avg;

    // Subtract the oldest data value in the window from the sum
    sum -= window[index];

    // Read new data from the ADC and update the sum
    window[index] = input;
    sum += window[index];

    // Move the window index
    index = (index + 1) %  SMA_WINDOW_SIZE;

    // Compute the average
    avg = sum /  SMA_WINDOW_SIZE;

    return avg;
}

float moving_average_filter_2(float input) {
    const int win_size = 15 ;

    static float window[win_size] = {0};
    static int index = 0;
    static double sum = 0;
    float avg;

    // Subtract the oldest data value in the window from the sum
    sum -= window[index];

    // Read new data from the ADC and update the sum
    window[index] = input;
    sum += window[index];

    // Move the window index
    index = (index + 1) % win_size;

    // Compute the average
    avg = sum / win_size;

    return avg;
}

// find max peak in one frame
float find_row_max(float array[]){
    float temp_max = 0 ;
    for(int i = 0 ; i < CONFIG_ROW_SIZE ; i++){
        if(array[i] > temp_max) temp_max = array[i];
    }
    return temp_max ;
}

