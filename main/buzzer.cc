#include "buzzer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "pinout_config.h"


#define TAG "BUZZER"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095

TaskHandle_t buzzerHandler = NULL ;

void _buzzer_config(uint32_t freq){
	// Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
		.duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = freq,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
		.gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
		.intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

//-------- interface function ---------

void tone(int gpio_num,uint32_t freq,uint32_t duration) {
	_buzzer_config(freq);
	// start
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY); 
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
	vTaskDelay(duration/portTICK_PERIOD_MS);
	// stop
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}


void play_start_sound(){
    tone(buzzer_pin, NOTE_CH, 100);
    vTaskDelay(10/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_DH, 100);
    vTaskDelay(10/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_EH, 300);
};

void play_restart_sound(){
    tone(buzzer_pin, NOTE_FH, 500);
};

void play_click_identified_sound(){
    tone(buzzer_pin, NOTE_BH, 50);
    vTaskDelay(50/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_FH, 50);
};

void play_inhale_identified_sound(){
    tone(buzzer_pin, NOTE_BH, 50);
    vTaskDelay(50/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_BH, 50);
};

void play_succeed_sound(){
    tone(buzzer_pin, NOTE_FH, 400);
    tone(buzzer_pin, NOTE_GH, 300);
    tone(buzzer_pin, NOTE_AH, 500);
    vTaskDelay(100/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_FH, 400);
    tone(buzzer_pin, NOTE_GH, 300);
    tone(buzzer_pin, NOTE_AH, 500);
};

void play_failed_sound(){
    tone(buzzer_pin, NOTE_CH, 100);
    vTaskDelay(50/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_CH, 600);
};

void play_powerOff_sound(){
    tone(buzzer_pin, NOTE_EH, 100);
    vTaskDelay(10/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_DH, 100);
    vTaskDelay(10/portTICK_PERIOD_MS);
    tone(buzzer_pin, NOTE_CH, 300);
};
