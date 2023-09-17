#ifndef _MESSAGE_PACKAGE_
#define _MESSAGE_PACKAGE_

/*
stop sign "/n"
1. stop (bool, 1/0)
2. press (bool, 1/0)
3. flowrate (uint8_t, [0,60]) Ex, 00, 01, 02, 03.....60
4. percentage (uint8_t)
5. which sequence (uint8_t, 0/1/2) enum

EX. stop:0press:1flowrate:00:sequence:0p:25\n
*/

enum {
    NOT_STOP = 0,
    YES_STOP
};

enum {
    NOT_PRESS = 0,
    YES_PRESS
};

enum {
    UNKNOWN_STATUS = 0,
    CLICK_BEFORE_INHALE,
    INHALE_BEFORE_CLICK
};

struct message_package_t{
    bool is_stop ;
    bool is_press ;
    int flowrate ;
    int percentage ;
    int sequence ;
};

void wait_for_go_command();
void send_start_message();
void send_message_package(message_package_t message);

#endif