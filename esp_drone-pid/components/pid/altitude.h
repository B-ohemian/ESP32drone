#ifndef   _ALTITUDE_H_
#define   _ALTITUDE_H_


typedef struct 
{
    float a;
    float v;
    float h;
    float last_h;
}cal_heigh;


void Strapdown_INS_High(float hight);
void reset_bro(void);

#endif
