#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <stdio.h>

#define NOTE_C 261
#define NOTE_D 294
#define NOTE_E 329
#define NOTE_F 349
#define NOTE_G 391
#define NOTE_GS 415
#define NOTE_A 440
#define NOTE_AS 455
#define NOTE_B 466
#define NOTE_CH 523
#define NOTE_CSH 554
#define NOTE_DH 587
#define NOTE_DSH 622
#define NOTE_EH 659
#define NOTE_FH 698
#define NOTE_FSH 740
#define NOTE_GH 784
#define NOTE_GSH 830
#define NOTE_AH 880
#define NOTE_ASH 932
#define NOTE_BH 987
#define NOTE_CHH 1046
#define NOTE_CSHH 1108
#define NOTE_DHH 1174

void tone(int gpio_num,uint32_t freq,uint32_t duration);

void play_start_sound();
void play_restart_sound();
void play_succeed_sound();
void play_failed_sound();
void play_powerOff_sound();
void play_inhale_identified_sound();
void play_click_identified_sound();

#endif