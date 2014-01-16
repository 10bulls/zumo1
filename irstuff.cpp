#include "irstuff.h"

int IRButtonMap(unsigned int ircode)
{
  switch (ircode)
  {
    case 0x240:
      return BUTTON_NUM_0;
    case 0x241:
      return BUTTON_NUM_1;
    case 0x242:
      return BUTTON_NUM_2;
    case 0x243:
      return BUTTON_NUM_3;
    case 0x244:
      return BUTTON_NUM_4;
    case 0x245:
      return BUTTON_NUM_5;
    case 0x246:
      return BUTTON_NUM_6;
    case 0x247:
      return BUTTON_NUM_7;
    case 0x248:
      return BUTTON_NUM_8;
    case 0x249:
      return BUTTON_NUM_9;
    case 0x24a:
      return BUTTON_MUTE;
    case 0x24c:
      return BUTTON_TEXT;
    case 0x24d:
      return BUTTON_CANCEL;
    case 0x24e:  // TV/AV
      return BUTTON_TV_AV;
    case 0x24f:
      return BUTTON_SUB;
    case 0x250:
      return BUTTON_LEFT;
    case 0x251:
      return BUTTON_RIGHT;
    case 0x252:
      return BUTTON_UP;
    case 0x253:
      return BUTTON_DOWN;
    case 0x254:
      return BUTTON_VOL_UP;
    case 0x255:
      return BUTTON_VOL_DOWN;
    case 0x256:  // CH+
      return BUTTON_CH_UP;
    case 0x257:  // CH-
      return BUTTON_CH_DOWN;
    case 0x258:
      return BUTTON_RED;
    case 0x259:
      return BUTTON_GREEN;
    case 0x25a:
      return BUTTON_BLUE;
    case 0x25b:
      return BUTTON_YELLOW;
    case 0x25c:
      return BUTTON_MENU;
    case 0x25d:
      return BUTTON_GUIDE;
    case 0x25e:
      return BUTTON_CALENDAR;
    case 0x268: 
      return BUTTON_REW;
    case 0x269: 
      return BUTTON_PAUSE;
    case 0x26a: 
      return BUTTON_STOP;
    case 0x26b: 
      return BUTTON_PREV;
    case 0x26c: 
      return BUTTON_FWD;
    case 0x26d:
      return BUTTON_PLAY;
    case 0x26e:
      return BUTTON_REC;
    case 0x26f:
      return BUTTON_NEXT;
    case 0x270:
      return BUTTON_LIBRARY;
    case 0x272:
      return BUTTON_OK;
  }
}


