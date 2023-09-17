#ifndef _DEVICE_CONFIG_H_
#define _DEVICE_CONFIG_H_

/***
 * This header files contains all the setting and configurations of inhaler exclude settings of model
 * Ex. sample rate, sample size, pinout.....etc 
 * 
 * p.s. include globaly
****/



#define USB_DEBUG_MODE


// Audio Processing configuration
#define SAMPLE_RATE 40960
#define SAMPLE_SIZE 512

#define ROW_COUNT 8
#define BAND_SIZE 32

// FFT
#define freq_perBin_hz  (SAMPLE_RATE / SAMPLE_SIZE) ;
const int bin_perBand ((SAMPLE_SIZE / 2) / BAND_SIZE);

// simple moving average window size
#define SMA_WINDOW_SIZE 10

// inhale threshold
#define CLICK_SCORE_THRESHOLD 200
#define CLICK_CHANGE_RATE_THRESHOLD 2000

// time standby to turn off
#define POWER_ON_TIME 100

// prediction model for inhale dection, if higher than it, than it's inahle
#define PREDICTION_IS_INHALE_THRESHOLD 0.99

#define IGNORE_CYCLE_COUNT 10

#endif