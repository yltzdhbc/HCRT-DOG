
#ifndef __VCAN_H
#define __VCAN_H

#include "robocon.h"
void vcan_send_byte(uint8_t date);
void vcan_send_wave_form(void);

extern short  wave_form_data[6] ;
#endif

