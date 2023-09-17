#ifndef _STATUS_H_
#define _STATUS_H_

typedef enum{
  SYSTEM_START = 0x00,
  SYSTEM_RESTART,
  SYSTEM_INHALE_CLICK_ROTATE_DETECTION,
  SYSTEM_FIRST_DROP_DETECTION,
  SYSTEM_AFTER_CLICK_DURING_1_5_SEC,
  SYSTEM_AFTER_CLICK_1_5_SEC_SUMMARY,
  SYSTEM_INHALE_SUCCEED,
  SYSTEM_INHALE_FAILED,
  SYSTEM_SHUT_DOWN
} inhalar_system_state_t;

inhalar_system_state_t status = SYSTEM_START ;

#endif