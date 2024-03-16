#ifndef FILTER_H
#define FILTER_H

#include "structconfig.h"

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;

Butter_BufferData Butter_Buffer;


// Butter_BufferData Butter_Buffer_Feedback[3];
// //-----Butterworth变量-----//
// Butter_Parameter Butter_80HZ_Parameter_Acce = {
//     // 200hz---80hz
//     {1, 1.14298050254, 0.4128015980962},
//     {0.638945525159, 1.277891050318, 0.638945525159}};

// Butter_Parameter Butter_60HZ_Parameter_Acce = {
//     // 200hz---60hz
//     {1, 0.3695273773512, 0.1958157126558},
//     {0.3913357725018, 0.7826715450035, 0.3913357725018}};

// Butter_Parameter Butter_51HZ_Parameter_Acce = {
//     // 200hz---51hz
//     {1, 0.03680751639284, 0.1718123812701},
//     {0.3021549744157, 0.6043099488315, 0.3021549744157}};

// Butter_Parameter Butter_30HZ_Parameter_Acce = {
//     // 200hz---30hz
//     {1, -0.7477891782585, 0.272214937925},
//     {0.1311064399166, 0.2622128798333, 0.1311064399166}};
// Butter_Parameter Butter_20HZ_Parameter_Acce = {
//     // 200hz---20hz
//     {1, -1.14298050254, 0.4128015980962},
//     {0.06745527388907, 0.1349105477781, 0.06745527388907}};
// Butter_Parameter Butter_15HZ_Parameter_Acce = {
//     // 200hz---15hz
//     {1, -1.348967745253, 0.5139818942197},
//     {0.04125353724172, 0.08250707448344, 0.04125353724172}};

// Butter_Parameter Butter_10HZ_Parameter_Acce = {
//     // 200hz---10hz
//     {1, -1.561018075801, 0.6413515380576},
//     {0.02008336556421, 0.04016673112842, 0.02008336556421}};
// Butter_Parameter Butter_5HZ_Parameter_Acce = {
//     // 200hz---5hz
//     {1, -1.778631777825, 0.8008026466657},
//     {0.005542717210281, 0.01108543442056, 0.005542717210281}};

// Butter_Parameter Butter_2HZ_Parameter_Acce = {
//     // 200hz---2hz
//     {1, -1.911197067426, 0.9149758348014},
//     {0.0009446918438402, 0.00188938368768, 0.0009446918438402}};


float Low_Filter(float value);
void SortAver_Filter(float value,float *filter,uint8_t N);
void  SortAver_Filter1(float value,float *filter,uint8_t n);
void  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t N);
void Aver_FilterXYZ6(INT16_XYZ *acc,INT16_XYZ *gry,FLOAT_XYZ *Acc_filt,FLOAT_XYZ *Gry_filt,uint8_t N);
void Aver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t N);
void Aver_Filter(float data,float *filt_data,uint8_t n);
void Aver_Filter1(float data,float *filt_data,uint8_t n);
void presssureFilter(float* in, float* out);

void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);
float LPF2pApply_1(float sample);

void kalman_1(struct _1_ekf_filter *ekf,float input);

float LPButterworth(float curr_input, Butter_BufferData *Buffer, Butter_Parameter *Parameter);
#endif
