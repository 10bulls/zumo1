#ifndef irstuff_h
#define irstuff_h

#define BUTTON_UNKNOWN    0
#define BUTTON_1          1
#define BUTTON_2          2
#define BUTTON_3          3
#define BUTTON_4          4
#define BUTTON_5          5
#define BUTTON_POWER      100
#define BUTTON_MUTE       101
#define BUTTON_VOL_UP     102
#define BUTTON_VOL_DOWN   103
#define BUTTON_CH_UP      104
#define BUTTON_CH_DOWN    105
#define BUTTON_NUM_0      106
#define BUTTON_NUM_1      107
#define BUTTON_NUM_2      108
#define BUTTON_NUM_3      109
#define BUTTON_NUM_4      110
#define BUTTON_NUM_5      111
#define BUTTON_NUM_6      112
#define BUTTON_NUM_7      113
#define BUTTON_NUM_8      114
#define BUTTON_NUM_9      115
#define BUTTON_TEXT       116
#define BUTTON_CANCEL     117
#define BUTTON_TV_AV      118
#define BUTTON_SUB        119
#define BUTTON_UP         120
#define BUTTON_DOWN       121
#define BUTTON_LEFT       122
#define BUTTON_RIGHT      123
#define BUTTON_OK         124
#define BUTTON_RED        125
#define BUTTON_GREEN      126
#define BUTTON_YELLOW     127
#define BUTTON_BLUE       128
#define BUTTON_MENU       129
#define BUTTON_GUIDE      130
#define BUTTON_CALENDAR   131
#define BUTTON_REW        132
#define BUTTON_FWD        133
#define BUTTON_PLAY       134
#define BUTTON_PAUSE      135
#define BUTTON_STOP       136
#define BUTTON_REC        137
#define BUTTON_PREV       138
#define BUTTON_NEXT       139
#define BUTTON_LIBRARY    140


int IRButtonMap(unsigned int ircode);


#endif
