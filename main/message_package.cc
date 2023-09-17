#include "message_package.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/*
stop sign "/n"
1. stop (bool, 1/0)
2. press (bool, 1/0)
3. flowrate (uint8_t, [0,60]) Ex, 00, 01, 02, 03.....60
4. percentage (uint8_t)
5. which sequence (uint8_t, 0/1/2) enum

EX. stop:0press:1flowrate:00:sequence:0p:25\n
*/

const char go_command = 'g' ;
const char sned_text_command = 's' ;

const char startMessage[] = "Start\n" ;

const char stopMessagePrefix[] = "stop:" ;
const char pressMessagePrefix[] = "press:" ;
const char flowrateMessagePrefix[] = "flowrate:" ;
const char sequenceMessagePrefix[] = "sequence:" ;
const char percentageMessagePrefix[] = "p:" ;

char* _intToString(int num) {
    // Allocate space for the string (including the null terminator)
    char* result = (char*) malloc(3 * sizeof(char));
    if (!result) {
        printf("Memory allocation failed!\n");
        exit(1);
    }
    
    // Convert integer to string in the desired format
    snprintf(result, 3, "%02d", num);
    return result;
}

void _wait_for_send_text_command(){
    while(1){
      char in = getchar() ;
      if(in == sned_text_command){
          break;
      }
      in = ' ';
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//-----------------

void wait_for_go_command(){
    while(1){
      char in = getchar() ;
      if(in == go_command){
          break;
      }
      in = ' ';
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void send_start_message(){
    printf(startMessage);
};

void send_message_package(message_package_t message){

    bool curr_is_stop = message.is_stop ;
    bool curr_is_press = message.is_press ;
    int curr_flowrate = message.flowrate ;
    int curr_percentage = message.percentage ;
    int curr_sequence = message.sequence ;

    char *curr_flowrate_string = _intToString(curr_flowrate);
    char *curr_percentage_string = _intToString(curr_percentage);

    //_wait_for_send_text_command();

    printf("%s%d%s%d%s%s%s%d%s%s\n",  stopMessagePrefix, curr_is_stop, 
                                    pressMessagePrefix, curr_is_press,
                                    flowrateMessagePrefix, curr_flowrate_string,
                                    sequenceMessagePrefix, curr_sequence,
                                    percentageMessagePrefix, curr_percentage_string );

    free(curr_flowrate_string);
    free(curr_percentage_string);
};