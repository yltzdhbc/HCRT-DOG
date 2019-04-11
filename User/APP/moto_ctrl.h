#ifndef MOTO_CTRL_H
#define MOTO_CTRL_H

#include "robocon.h"

void moto_behaviour(void);
void moto_param_init(void);
void coordinate_trans(float X,float Y);

typedef struct
{
    float ref_agle[8];
    float out[8];
} temp_data;

extern temp_data temp_pid;
#endif
