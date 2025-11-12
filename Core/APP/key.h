#ifndef __KEY_H
#define __KEY_H

#include "mybsp.h"

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESSED,
    KEY_EVENT_RELEASED,
} Key_Event_Type;

void Keypad_FSM_Task(void);
Key_Event_Type Keypad_Get_Event(unsigned char *key_value);
//¶¨Òå°´¼ü
#define KEY_1 '1'
#define KEY_2 '2'
#define KEY_3 '3'
#define KEY_A 'A'
#define KEY_4 '4'
#define KEY_5 '5'
#define KEY_6 '6'
#define KEY_B 'B'
#define KEY_7 '7'
#define KEY_8 '8'
#define KEY_9 '9'
#define KEY_C 'C'
#define KEY_STAR '*'
#define KEY_0 '0'
#define KEY_HASH '#'
#define KEY_D 'D'
#define NO_KEY 0xFF
#endif