#include "pid.h"
#include "ledc_motor.h"
#include "imu.h"
#include "user_udp.h"
#include "altitude.h"
#include "battery.h"
#include "filter.h"

#define HEIGHT_EXPECT 100.0 // 定高期望高度

int16_t Z_THROTTLE = 400; // 基础油门

// 长电池的初始油门为630

// //************外环pid参数** 第四版************************//
// float PID_OUT_PITCH_KP = 4.0;				 // 2.5
// float PID_OUT_PITCH_KI = 0;					 // 0
// float PID_OUT_PITCH_KD = 0.5;				 // 0
// float PID_OUT_PITCH_INTEGRATION_LIMIT = 300; // 500.0

// float PID_OUT_ROLL_KP = 4.0;				// 3
// float PID_OUT_ROLL_KI = 0;					// 0
// float PID_OUT_ROLL_KD = 0.5;				// 0
// float PID_OUT_ROLL_INTEGRATION_LIMIT = 300; // 500.0

// //************内环pid参数**************************//
// float PID_IN_PITCH_KP = 0.5;				// 0.93
// float PID_IN_PITCH_KI = 0.0;				// 0.0
// float PID_IN_PITCH_KD = 0.8;				// 0.5
// float PID_IN_PITCH_INTEGRATION_LIMIT = 300; // 500.0 积分限幅的值会影响静态误差

// float PID_IN_ROLL_KP = 0.5;				   // 0.93
// float PID_IN_ROLL_KI = 0.0;				   // 0.0
// float PID_IN_ROLL_KD = 0.8;				   // 0.5
// float PID_IN_ROLL_INTEGRATION_LIMIT = 300; // 500.0

// float PID_IN_YAW_KP = 9;				  // 2
// float PID_IN_YAW_KI = 0.01;				  // 0.05
// float PID_IN_YAW_KD = 0.5;				  // 1
// float PID_IN_YAW_INTEGRATION_LIMIT = 300; // 300.0

//************外环pid参数****第五版**********************//
// float PID_OUT_PITCH_KP = 0.5;				 // 2.5
// float PID_OUT_PITCH_KI = 0;					 // 0
// float PID_OUT_PITCH_KD = 0.1;				 // 0
// float PID_OUT_PITCH_INTEGRATION_LIMIT = 300; // 500.0

// float PID_OUT_ROLL_KP = 0.5;				// 3
// float PID_OUT_ROLL_KI = 0;					// 0
// float PID_OUT_ROLL_KD = 0.1;				// 0
// float PID_OUT_ROLL_INTEGRATION_LIMIT = 300; // 500.0

//************内环pid参数**************************//
// float PID_IN_PITCH_KP = 1.30;				// 0.93
// float PID_IN_PITCH_KI = 0.0;				// 0.0
// float PID_IN_PITCH_KD = 0.8;				// 0.5
// float PID_IN_PITCH_INTEGRATION_LIMIT = 300; // 500.0 积分限幅的值会影响静态误差

// float PID_IN_ROLL_KP = 1.30;				   // 0.93
// float PID_IN_ROLL_KI = 0.0;				   // 0.0
// float PID_IN_ROLL_KD = 0.8;				   // 0.5
// float PID_IN_ROLL_INTEGRATION_LIMIT = 300; // 500.0

// float PID_IN_YAW_KP = 9;				  // 2
// float PID_IN_YAW_KI = 0.01;				  // 0.05
// float PID_IN_YAW_KD = 0.5;				  // 1
// float PID_IN_YAW_INTEGRATION_LIMIT = 300; // 300.0

//************外环pid参数****3D打印版**********************//
float PID_OUT_PITCH_KP = 0.8;				 // 2.5
float PID_OUT_PITCH_KI = 0;					 // 0
float PID_OUT_PITCH_KD = 0.1;				 // 0
float PID_OUT_PITCH_INTEGRATION_LIMIT = 300; // 500.0

float PID_OUT_ROLL_KP = 0.8;				// 3
float PID_OUT_ROLL_KI = 0;					// 0
float PID_OUT_ROLL_KD = 0.1;				// 0
float PID_OUT_ROLL_INTEGRATION_LIMIT = 300; // 500.0

//************内环pid参数**************************//
float PID_IN_PITCH_KP = 1.15;				// 0.93
float PID_IN_PITCH_KI = 0.0;				// 0.0
float PID_IN_PITCH_KD = 1.20;				// 0.5
float PID_IN_PITCH_INTEGRATION_LIMIT = 300; // 500.0 积分限幅的值会影响静态误差

float PID_IN_ROLL_KP = 1.15;				   // 0.93
float PID_IN_ROLL_KI = 0.0;				   // 0.0
float PID_IN_ROLL_KD = 1.20;				   // 0.5
float PID_IN_ROLL_INTEGRATION_LIMIT = 300; // 500.0

float PID_IN_YAW_KP = 9;				  // 2
float PID_IN_YAW_KI = 0.01;				  // 0.05
float PID_IN_YAW_KD = 0.5;				  // 1
float PID_IN_YAW_INTEGRATION_LIMIT = 300; // 300.0


// float PID_OUT_PITCH_KP = 2.0;					 // 2.5
// float PID_OUT_PITCH_KI = 0;					 // 0
// float PID_OUT_PITCH_KD = 0;				 // 0
// float PID_OUT_PITCH_INTEGRATION_LIMIT = 300; // 500.0

// float PID_OUT_ROLL_KP = 2.0;					// 3
// float PID_OUT_ROLL_KI = 0;					// 0
// float PID_OUT_ROLL_KD = 0;				// 0
// float PID_OUT_ROLL_INTEGRATION_LIMIT = 300; // 500.0

// //************内环pid参数**************************//
// float PID_IN_PITCH_KP = 2.0;				// 0.5
// float PID_IN_PITCH_KI = 0.0;				// 0.003
// float PID_IN_PITCH_KD = 1.5;				// 0.25
// float PID_IN_PITCH_INTEGRATION_LIMIT = 300; // 500.0 积分限幅的值会影响静态误差

// float PID_IN_ROLL_KP = 2.0;			   // 0.3
// float PID_IN_ROLL_KI = 0.0;				   // 0.003
// float PID_IN_ROLL_KD = 1.5;				   // 0.20
// float PID_IN_ROLL_INTEGRATION_LIMIT = 300; // 500.0

// float PID_IN_YAW_KP = 2;				  // 3.0
// float PID_IN_YAW_KI = 0.05;				  // 0
// float PID_IN_YAW_KD = 1;				  // 0
// float PID_IN_YAW_INTEGRATION_LIMIT = 300; // 200.0

////////////////////////////////////////////////////////////////////

// 高度——加速度环
float PID_IN_ACC_Z_KP = 1.0;				// 3.0
float PID_IN_ACC_Z_KI = 0.0;				// 0
float PID_IN_ACC_Z_KD = 1.5;				// 0
float PID_IN_ACC_Z_INTEGRATION_LIMIT = 300; // 200.0
// 高度——速度环
float PID_IN_Z_KP = 2.0;				// 2.0
float PID_IN_Z_KI = 0.05;				// 0
float PID_IN_Z_KD = 9.0;				// 0
float PID_IN_Z_INTEGRATION_LIMIT = 300; // 200.0
// 高度——高度环
float PID_OUT_Z_KP = 0.1;				 // 3.0
float PID_OUT_Z_KI = 0.001;				 // 0
float PID_OUT_Z_KD = 9.0;				 // 0
float PID_OUT_Z_INTEGRATION_LIMIT = 300; // 200.0

extern bool FLAG;
extern alt_bro ALT_BRO;
extern alt_acc ALT_ACC;
extern cal_heigh CAL_HEIGH;

bool MODE = 0;

/*
// x:pitch    y:roll    z:yaw

//              y
//              |
//           1  |  2
//            \ | /
//             \ /
//              * ------x
//             / \
//            /   \
//           4     3
*/
uint8_t armed = 1;
float MOTO1_PWM = 0.0f, MOTO2_PWM = 0.0f, MOTO3_PWM = 0.0f, MOTO4_PWM = 0.0f; // 四电机pwm

int16_t RC_PIT = 0;
int16_t RC_ROL = 0;
int16_t RC_YAW = 0;
int16_t RC_THROTTLE = 0; // 初始油门

pidsuite pid_out_Pitch; // 俯仰角外环pid
pidsuite pid_out_Roll;	// 横滚角外环pid
pidsuite pid_in_Pitch;	// 俯仰角内环pid
pidsuite pid_in_Roll;	// 横滚角内环pid
pidsuite pid_in_Yaw;	// 偏航角内环pid

pidsuite pid_in_Z;
pidsuite pid_out_Z;
pidsuite pid_in_Z;
pidsuite pid_in_acc_Z;

piddata getpiddata; // 接收来自遥控器的pid值

float pid_out_pitch = 0; // 输出值
float pid_out_roll = 0;
float pid_in_pitch = 0;
float pid_in_roll = 0;
float pid_in_yaw = 0;

float pid_in_z = 0;
float pid_out_z = 0;
float pid_in_acc_z = 0;

float Z_THRUST = 0;

extern float offset_pit, offset_rol;
/*------------------------------------------pid结构初始化-----------------------------------------*/
// 输入参数：结构体指针，期望值，kp,ki,kd
void pidInit(pidsuite *pid, const float desired, const float kp,
			 const float ki, const float kd)
{

	pid->error = 0;
	pid->prevError = 0;
	pid->integ = 0;
	pid->deriv = 0; // 微分
	pid->desired = desired;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
}

/*----------------------------------------------pid积分部分限制值-------------------------------------------*/
void pidSetIntegralLimit(pidsuite *pid, const float limit)
{
	pid->iLimit = limit;
}

void PID_controllerInit(void) // pid参数初始化
{

	pidInit(&pid_out_Pitch, 0, PID_OUT_PITCH_KP, PID_OUT_PITCH_KI, PID_OUT_PITCH_KD);
	pidInit(&pid_out_Roll, 0, PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD);

	pidInit(&pid_in_Pitch, 0, PID_IN_PITCH_KP, PID_IN_PITCH_KI, PID_IN_PITCH_KD);
	pidInit(&pid_in_Roll, 0, PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD);
	pidInit(&pid_in_Yaw, 0, PID_IN_YAW_KP, PID_IN_YAW_KI, PID_IN_YAW_KD);

	pidInit(&pid_in_Z, 0, PID_IN_Z_KP, PID_IN_Z_KI, PID_IN_Z_KD);
	pidInit(&pid_out_Z, 0, PID_OUT_Z_KP, PID_OUT_Z_KI, PID_OUT_Z_KD);
	pidInit(&pid_in_acc_Z, 0, PID_IN_ACC_Z_KP, PID_IN_ACC_Z_KI, PID_IN_ACC_Z_KD);

	pidSetIntegralLimit(&pid_out_Pitch, PID_OUT_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_out_Roll, PID_OUT_ROLL_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_Pitch, PID_IN_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_Roll, PID_IN_ROLL_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_Yaw, PID_IN_YAW_INTEGRATION_LIMIT);

	pidSetIntegralLimit(&pid_in_Z, PID_IN_Z_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_out_Z, PID_OUT_Z_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_acc_Z, PID_IN_ACC_Z_INTEGRATION_LIMIT);
	// printf("pid inti!\n");
}

/*--------------------------------------pid输出更新------------------------------------------*/
// 输入参数：pid结构体指针，测量值 ,期望值
// 输出：pid输出
float pidUpdate(pidsuite *pid, const float measured, float expect)
{
	float output;

	pid->desired = expect; // 获取期望角度

	pid->error = pid->desired - measured; // 偏差：期望-测量值

	pid->integ += pid->error; // 偏差积分

	if (pid->integ > pid->iLimit) // 积分限幅
	{
		pid->integ = pid->iLimit;
	}
	else if (pid->integ < -pid->iLimit) // 积分限幅
	{
		pid->integ = -pid->iLimit;
	}
	pid->deriv = pid->error - pid->prevError; // 微分

	pid->outP = pid->kp * pid->error; // 方便独立观察
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP +
			 pid->outI +
			 pid->outD;

	pid->prevError = pid->error; // 更新前一次偏差

	return output;
}
#define LPF_3_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz)*6.28f * (t)))) * ((in) - (out)))
float fliterout1, fliterout2;
Butter_BufferData Butter_Buffer2;
Butter_BufferData Butter_Buffer3;

Butter_Parameter Butter_20HZ_Parameter_Acce = {
	// 200hz---20hz
	{-0.00777631271910257, 0.06445464557871, 0.443321667140393},
	{0.443321667140393, 0.06445464557871, -0.00777631271910257}};

void Control(uint8_t armed)
{
	float pitch_expect = (float)RC_PIT / 40.0f ;	 // 最大值1950左右
	float roll_expect = -(float)RC_ROL / 40.0f ; // 限制在45°
	float yaw_expect = (float)RC_YAW / 40.0f;			 //


	//-------------------------------------------------------------------从这里注释掉调参的参数显示-------------------------------------------------------------------
	// printf("pidincont:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD, PID_IN_PITCH_KP, PID_IN_PITCH_KI,
		  //  PID_IN_PITCH_KD, PID_IN_YAW_KP, PID_IN_YAW_KI, PID_IN_YAW_KD, PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD, PID_OUT_PITCH_KP, PID_OUT_PITCH_KI,
		  //  PID_OUT_PITCH_KD, PID_IN_Z_KP, PID_IN_Z_KI, PID_IN_Z_KD, PID_OUT_Z_KP, PID_OUT_Z_KI, PID_OUT_Z_KD,PID_IN_ACC_Z_KP,PID_IN_ACC_Z_KI,PID_IN_ACC_Z_KD);

	LPButterworth(Att_Angle.pit, &Butter_Buffer2, &Butter_20HZ_Parameter_Acce);
	LPButterworth(Att_Angle.rol, &Butter_Buffer3, &Butter_20HZ_Parameter_Acce);
	// printf("struct:%f,%f,%f\n",Att_Angle.pit,Butter_Buffer2.Output_Butter[2],fliterout1);
	// printf("struct:%f,%f,%f,%f\n",pitch_expect,roll_expect,Att_Angle.pit,Att_Angle.rol);

	if (RC_THROTTLE > YM_Dead) // 油门大于死区才进行控制,油门的最大值为700，留200给PID调整
	{
		// 外环输入是角度，传感器测的是角度，输出的是角速度，

		pid_out_pitch = pidUpdate(&pid_out_Pitch, Butter_Buffer2.Output_Butter[2], pitch_expect);
		pid_out_roll = pidUpdate(&pid_out_Roll, Butter_Buffer3.Output_Butter[2], roll_expect);

		// 内环输入是外环的输出，即角速度，传感器测的是陀螺仪的角速度，
		pid_in_pitch = pidUpdate(&pid_in_Pitch, (Gyr_rad.Y * RadtoDeg), pid_out_pitch); // 内环是角速度
		pid_in_roll = pidUpdate(&pid_in_Roll, -(Gyr_rad.X * RadtoDeg), pid_out_roll);
		pid_in_yaw = pidUpdate(&pid_in_Yaw, (Gyr_rad.Z * RadtoDeg), yaw_expect);
	}
	else if (RC_THROTTLE < YM_Dead) // 否则积分清零
	{
		pid_in_Pitch.integ = 0;
		pid_in_Roll.integ = 0;
		pid_in_Yaw.integ = 0;
	}
	if (RC_THROTTLE > YM_Dead) // 相当于一个启动开关
	{
		if (MODE == 0) // 默认为自稳模式
		{
			// // 自稳动力分配
			MOTO1_PWM = RC_THROTTLE + pid_in_pitch - pid_in_roll - pid_in_yaw;
			MOTO2_PWM = RC_THROTTLE - pid_in_pitch - pid_in_roll + pid_in_yaw;
			MOTO3_PWM = RC_THROTTLE - pid_in_pitch + pid_in_roll - pid_in_yaw;
			MOTO4_PWM = RC_THROTTLE + pid_in_pitch + pid_in_roll + pid_in_yaw;
		}
		else
		{
			//  定高油门分配
			MOTO1_PWM = Z_THRUST * 1.2 + RC_THROTTLE + pid_in_pitch - pid_in_roll - pid_in_yaw;
			MOTO2_PWM = Z_THRUST * 1.2 + RC_THROTTLE - pid_in_pitch - pid_in_roll + pid_in_yaw;
			MOTO3_PWM = Z_THRUST * 1.2 + RC_THROTTLE - pid_in_pitch + pid_in_roll - pid_in_yaw;
			MOTO4_PWM = Z_THRUST * 1.2 + RC_THROTTLE + pid_in_pitch + pid_in_roll + pid_in_yaw;
			// printf("d:%f,%f,%f,%f\n",MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM);
		}
	}

	if (RC_THROTTLE > YM_Dead)
	{ // 如果油门大于死区值，PWM的最低值为死区值，最高值为100
		// 限幅
		if (MOTO1_PWM < 0)
			MOTO1_PWM = 0;
		else if (MOTO1_PWM > 900)
			MOTO1_PWM = 900;
		if (MOTO2_PWM < 0)
			MOTO2_PWM = 0;
		else if (MOTO2_PWM > 900)
			MOTO2_PWM = 900;
		if (MOTO3_PWM < 0)
			MOTO3_PWM = 0;
		else if (MOTO3_PWM > 900)
			MOTO3_PWM = 900;
		if (MOTO4_PWM < 0)
			MOTO4_PWM = 0;
		else if (MOTO4_PWM > 900)
			MOTO4_PWM = 900;
	}
	else // 否则让电机的PWM等于油门值，逐步起飞
	{
		MOTO1_PWM = 0, MOTO2_PWM = 0, MOTO3_PWM = 0, MOTO4_PWM = 0;
	}

	ledc_motor_run((int)MOTO1_PWM, (int)MOTO2_PWM, (int)MOTO3_PWM, (int)MOTO4_PWM);
}

// 悬停油门的基础值
int16_t estimateHoverThru(void)
{
	float hoverHru = 0;
	float battery = 0;

	// 电池电压检测
	battery = get_battery_adc();

	if (battery > 4.05)
	{
		hoverHru = 400;
	}
	else if (battery > 3.90)
	{
		hoverHru = 430;
	}
	else if (battery > 3.80)
	{
		hoverHru = 480;
	}
	else if (battery > 3.70)
	{
		hoverHru = 530;
	}
	else
	{
		hoverHru = 550;
	}

	return hoverHru;
}

#define LPF_2_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz)*6.28f * (t)))) * ((in) - (out)))
float high_fliter_acc;
// 定高PID控制
void PID_Z()
{
	 LPF_2_(10,0.001,CAL_HEIGH.a,high_fliter_acc);
	//  printf("filter:%f,%f,%f,%f\n",CAL_HEIGH.a,high_fliter_acc,CAL_HEIGH.v,CAL_HEIGH.h);
	pid_out_z = pidUpdate(&pid_out_Z, CAL_HEIGH.h, HEIGHT_EXPECT);
	// Z_THRUST=pidUpdate(&pid_out_Z, CAL_HEIGH.h, HEIGHT_EXPECT);
	Z_THRUST = pidUpdate(&pid_in_Z, CAL_HEIGH.v, pid_out_z);
	//  Z_THRUST=pidUpdate(&pid_in_acc_Z, high_fliter_acc, pid_in_z);
	//  Z_THRUST= pidUpdate(&pid_out_Z, CAL_HEIGH.h, HEIGHT_EXPECT);
	// printf("z_thrust:%f,%f,%f\n", pid_in_z,CAL_HEIGH.h,CAL_HEIGH.v);
}

void pid_change(uint8_t pid[96])
{
	for (uint8_t i = 0; i < 96; i++)
	{
		pid[i] = pid[i] - 48;
	}
	PID_IN_ROLL_KP = (pid[0] * 100 + pid[1] * 10 + pid[2]) / 100.0;
	PID_IN_ROLL_KI = (pid[4] * 100 + pid[5] * 10 + pid[6]) / 100.0;
	PID_IN_ROLL_KD = (pid[8] * 100 + pid[9] * 10 + pid[10]) / 100.0;

	PID_IN_PITCH_KP = (pid[12] * 100 + pid[13] * 10 + pid[14]) / 100.0;
	PID_IN_PITCH_KI = (pid[16] * 100 + pid[17] * 10 + pid[18]) / 100.0;
	PID_IN_PITCH_KD = (pid[20] * 100 + pid[21] * 10 + pid[22]) / 100.0;

	PID_IN_YAW_KP = (pid[24] * 100 + pid[25] * 10 + pid[26]) / 100.0;
	PID_IN_YAW_KI = (pid[28] * 100 + pid[29] * 10 + pid[30]) / 100.0;
	PID_IN_YAW_KD = (pid[32] * 100 + pid[33] * 10 + pid[34]) / 100.0;

	PID_OUT_ROLL_KP = (pid[36] * 100 + pid[37] * 10 + pid[38]) / 100.0;
	PID_OUT_ROLL_KI = (pid[40] * 100 + pid[41] * 10 + pid[42]) / 100.0;
	PID_OUT_ROLL_KD = (pid[44] * 100 + pid[45] * 10 + pid[46]) / 100.0;

	PID_OUT_PITCH_KP = (pid[48] * 100 + pid[49] * 10 + pid[50]) / 100.0;
	PID_OUT_PITCH_KI = (pid[52] * 100 + pid[53] * 10 + pid[54]) / 100.0;
	PID_OUT_PITCH_KD = (pid[56] * 100 + pid[57] * 10 + pid[58]) / 100.0;

	// 高度——速度环
	PID_IN_Z_KP = (pid[60] * 100 + pid[61] * 10 + pid[62]) / 10.0;
	PID_IN_Z_KI = (pid[64] * 100 + pid[65] * 10 + pid[66]) / 10.0;
	PID_IN_Z_KD = (pid[68] * 100 + pid[69] * 10 + pid[70]) / 10.0;

	// 高度——高度环
	PID_OUT_Z_KP = (pid[72] * 100 + pid[73] * 10 + pid[74]) / 10.0;
	PID_OUT_Z_KI = (pid[76] * 100 + pid[77] * 10 + pid[78]) / 10.0;
	PID_OUT_Z_KD = (pid[80] * 100 + pid[81] * 10 + pid[82]) / 10.0;

	// 高度——加速度环
	PID_IN_ACC_Z_KP = (pid[84] * 100 + pid[85] * 10 + pid[86]) / 1000.0;
	PID_IN_ACC_Z_KI = (pid[88] * 100 + pid[89] * 10 + pid[90]) / 10.0;
	PID_IN_ACC_Z_KD = (pid[92] * 100 + pid[93] * 10 + pid[94]) / 10.0;


	pidInit(&pid_out_Pitch, 0, PID_OUT_PITCH_KP, PID_OUT_PITCH_KI, PID_OUT_PITCH_KD);
	pidInit(&pid_out_Roll, 0, PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD);

	pidInit(&pid_in_Pitch, 0, PID_IN_PITCH_KP, PID_IN_PITCH_KI, PID_IN_PITCH_KD);
	pidInit(&pid_in_Roll, 0, PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD);
	pidInit(&pid_in_Yaw, 0, PID_IN_YAW_KP, PID_IN_YAW_KI, PID_IN_YAW_KD);

	pidInit(&pid_in_Z, 0, PID_IN_Z_KP, PID_IN_Z_KI, PID_IN_Z_KD);
	pidInit(&pid_out_Z, 0, PID_OUT_Z_KP, PID_OUT_Z_KI, PID_OUT_Z_KD);
	pidInit(&pid_in_acc_Z, 0, PID_IN_ACC_Z_KP, PID_IN_ACC_Z_KI, PID_IN_ACC_Z_KD);

	pidSetIntegralLimit(&pid_out_Pitch, PID_OUT_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_out_Roll, PID_OUT_ROLL_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_Pitch, PID_IN_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_Roll, PID_IN_ROLL_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_Yaw, PID_IN_YAW_INTEGRATION_LIMIT);

	pidSetIntegralLimit(&pid_in_Z, PID_IN_Z_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_out_Z, PID_OUT_Z_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_in_acc_Z, PID_IN_ACC_Z_INTEGRATION_LIMIT);

	// printf("pid:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",PID_IN_ROLL_KP,PID_IN_ROLL_KI,PID_IN_ROLL_KD,PID_IN_PITCH_KP,PID_IN_PITCH_KI,
	// PID_IN_PITCH_KD,PID_IN_YAW_KP,PID_IN_YAW_KI,PID_IN_YAW_KD,PID_OUT_ROLL_KP,PID_OUT_ROLL_KI,PID_OUT_ROLL_KD,PID_OUT_PITCH_KP,PID_OUT_PITCH_KI,
	// PID_OUT_PITCH_KD);
}

// 接收遥控数据的任务
void argument_receive_task(void *arg)
{
	QueueHandle_t QHandle_MOTOR;
	QHandle_MOTOR = (QueueHandle_t)arg;
	BaseType_t xStaus;

	float pid_arg[18] = {0};
	while (1)
	{
		if (uxQueueMessagesWaiting(QHandle_MOTOR) != 0)
		{
			xStaus = xQueueReceive(QHandle_MOTOR, &pid_arg, 0);
			if (xStaus != pdPASS)
			{
				// printf("recv faild!\n");
			}
			else
			{
				PID_IN_ROLL_KP = pid_arg[0];
				PID_IN_ROLL_KI = pid_arg[1];
				PID_IN_ROLL_KD = pid_arg[2];

				PID_IN_PITCH_KP = pid_arg[3];
				PID_IN_PITCH_KI = pid_arg[4];
				PID_IN_PITCH_KD = pid_arg[5];

				PID_IN_YAW_KP = pid_arg[6];
				PID_IN_YAW_KI = pid_arg[7];
				PID_IN_YAW_KD = pid_arg[8];

				PID_OUT_ROLL_KP = pid_arg[9];
				PID_OUT_ROLL_KI = pid_arg[10];
				PID_OUT_ROLL_KD = pid_arg[11];

				PID_OUT_PITCH_KP = pid_arg[12];
				PID_OUT_PITCH_KI = pid_arg[13];
				PID_OUT_PITCH_KD = pid_arg[14];

				pidInit(&pid_out_Pitch, 0, PID_OUT_PITCH_KP, PID_OUT_PITCH_KI, PID_OUT_PITCH_KD);
				pidInit(&pid_out_Roll, 0, PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD);

				pidInit(&pid_in_Pitch, 0, PID_IN_PITCH_KP, PID_IN_PITCH_KI, PID_IN_PITCH_KD);
				pidInit(&pid_in_Roll, 0, PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD);
				pidInit(&pid_in_Yaw, 0, PID_IN_YAW_KP, PID_IN_YAW_KI, PID_IN_YAW_KD);

				pidSetIntegralLimit(&pid_out_Pitch, PID_OUT_PITCH_INTEGRATION_LIMIT);
				pidSetIntegralLimit(&pid_out_Roll, PID_OUT_ROLL_INTEGRATION_LIMIT);
				pidSetIntegralLimit(&pid_in_Pitch, PID_IN_PITCH_INTEGRATION_LIMIT);
				pidSetIntegralLimit(&pid_in_Roll, PID_IN_ROLL_INTEGRATION_LIMIT);
				pidSetIntegralLimit(&pid_in_Yaw, PID_IN_YAW_INTEGRATION_LIMIT);

				// printf("----------------------------------------------\n");
				// printf("P1:%f,I1:%f,D1:%f\n", PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD);
				// printf("P2:%f,I2:%f,D2:%f\n", PID_IN_PITCH_KP, PID_IN_PITCH_KI, PID_IN_PITCH_KD);
				// printf("P3:%f,I3:%f,D3:%f\n", PID_IN_YAW_KP, PID_IN_YAW_KI, PID_IN_YAW_KD);
				// printf("P4:%f,I4:%f,D4:%f\n", PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD);
				// printf("P5:%f,I5:%f,D5:%f\n", PID_OUT_PITCH_KP, PID_OUT_PITCH_KI, PID_OUT_PITCH_KD);
				// printf("P6:%f,I6:%f,D6:%f,\n", RC_PIT, RC_ROL, RC_YAW);
				// printf("----------------------------------------------\n");
			}
		}
		vTaskDelay(1 / portTICK_RATE_MS);
	}
}
