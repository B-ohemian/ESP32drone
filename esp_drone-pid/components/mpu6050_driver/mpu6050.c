#include "mpu6050.h"
#include "pid.h"
#include "imu.h"
#include "esp_log.h"
#include "spl06.h"
#include "altitude.h"
#include "battery.h"
#include "supersconic.h"

#define SCL 1 /*!< I2C SCL pin number  */
#define SDA 2 /*!< I2C SDA pin number  */

// #define SCL 14 /*!< I2C SCL pin number  */
// #define SDA 13 /*!< I2C SDA pin number  */

static uint8_t MPU6050_buff[14];								   // 加速度 陀螺仪 温度 原始数据
INT16_XYZ GYRO_OFFSET_RAW, ACC_OFFSET_RAW, OFFSET_ACC, OFFSET_GRO; // 零漂数据
INT16_XYZ MPU6050_ACC_RAW, MPU6050_GYRO_RAW;					   // 读取值原始数据
uint8_t SENSER_OFFSET_FLAG;										   // 传感器校准标志位

uint8_t OFFSET_FLAG = 0; // 气压计矫正完成标志位

int32_t offset_acc_x = 0, offset_acc_y = 0, offset_acc_z = 0;
int32_t offset_gro_x = 0, offset_gro_y = 0, offset_gro_z = 0;

float offset_pit = 0, offset_rol = 0;

/**
 * I2C初始化
 */
void I2C_Init()
{
	esp_err_t esp_err;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA, // select GPIO specific to your project
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = SCL, // select GPIO specific to your project
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 200000, // select frequency specific to your project
									// .clk_flags = 0,
	};
	esp_err = i2c_param_config(0, &conf);
	printf("i2c_param_config: %d \n", esp_err);

	esp_err = i2c_driver_install(0, I2C_MODE_MASTER, 0, 0, 0);
	printf("i2c_driver_install: %d \n", esp_err);
}

/**
 * @brief MPU-6050 initial
 */
uint8_t MPU_Init()
{
	uint8_t res;
	// 初始化ESP32硬件I2C
	// I2C_Init();
	// 等待初始化完成
	// vTaskDelay(200 / portTICK_RATE_MS);
	// 复位MPU6050, 寄存器0x6B bit7写1实现
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);
	vTaskDelay(200 / portTICK_RATE_MS);
	// 唤醒MPU6050
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);
	// 设置陀螺仪满量程,2000dps
	MPU_Set_Gyro_FSR(3);
	// 设置加速度计满量程范围,+-4g
	MPU_Set_Accel_FSR(1);
	// 设置采样率
	MPU_Set_Rate(200);
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	 // 关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 // 关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
	MPU_Read_Byte(MPU_DEVICE_ID_REG, &res);

	// MPU_Read_Byte(0x0d, &res);
	printf("add:%x\n", res);
	if (res == MPU_ADDR) // 器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
		MPU_Set_Rate(200);						 // 设置采样率为50Hz
												 // printf("add:%x\n",res);
	}
	else
		return 1;

	return 0;
}

/*****************************************************************************
 * 函  数：uint8_t MPU6050_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
 * 功  能：从指定寄存器写入指定长度数据
 * 参  数：reg：寄存器地址
 *         len：写入数据长度
 *         buf: 写入数据存放的地址
 * 返回值：0成功 1失败
 * 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
 *****************************************************************************/
uint8_t MPU6050_WriteMultBytes(uint8_t reg, uint8_t len, uint8_t *buf)
{
	if (MPU_Write_Len(reg, buf, len))
		return 1;
	else
		return 0;
}

/*****************************************************************************
 * 函  数：uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
 * 功  能：从指定寄存器读取指定长度数据
 * 参  数：reg：寄存器地址
 *         len：读取数据长度
 *         buf: 读取数据存放的地址
 * 返回值：0成功 0失败
 * 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
 *****************************************************************************/
uint8_t MPU6050_ReadMultBytes(uint8_t reg, uint8_t len, uint8_t *buf)
{
	if (MPU_Read_Len(reg, buf, len))
		return 1;
	else
		return 0;
}

/**
 * @brief Set the Gyroscope full-scale range of ±250, ±500, ±1000, and ±2000°/sec (dps)
 *
 * @param fsr the number of register, it could be 0, 1, 2, 3
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Gyro_FSR(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

/**
 * @brief Set the Accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g
 *
 * @param fsr the number of register, it could be 0, 1, 2, 3
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Accel_FSR(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/**
 * @brief Set the Sample rate of Gyroscope, Accelerometer, DMP, etc.
 *
 * @param rate parameter is the sample rate of Gyroscope, Accelerometer, DMP, etc.
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);
	return MPU_Set_LPF(rate / 2); /*!< set low pass filter the half of the rate */
}

/**
 * @brief 通过I2C写一个字节数据到MPU6050
 *
 * @param reg parameter is a register of MPU-6050 //寄存器地址
 * @param data parameter will be written to the register of MPU-6050 //将要写入寄存器的数据
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	esp_err_t error;
	// 创建I2C连接，返回连接句柄
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	// 写启动信号
	i2c_master_start(cmd);
	// 广播地址，并说明读写，并等待响应
	error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, 1);
	if (error != ESP_OK)
		return 1;
	// 写需要写入数据的寄存器地址
	error = i2c_master_write_byte(cmd, reg, 1);
	if (error != ESP_OK)
		return 1;
	// 向上一步指定的寄存器写入数据。
	error = i2c_master_write_byte(cmd, data, 1);
	if (error != ESP_OK)
		return 1;
	// 停止
	i2c_master_stop(cmd);
	// 发送数据
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	// 删除连接
	i2c_cmd_link_delete(cmd);

	return 0;
}

/**
 * @brief Write a buffer to MPU-6050 through I2C
 *
 * @param reg parameter is a register of MPU-6050
 * @param data parameter is a buffer which will be written to a register of MPU-6050
 * @param len parameter is the length of data
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Write_Len(uint8_t reg, uint8_t *data, uint8_t len)
{
	esp_err_t error;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, 1);
	if (error != ESP_OK)
		return 1;

	error = i2c_master_write_byte(cmd, reg, 1);
	if (error != ESP_OK)
		return 1;
	// 将缓存区数据写入寄存器
	error = i2c_master_write(cmd, data, len, 1);
	if (error != ESP_OK)
		return 1;

	i2c_master_stop(cmd);

	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);

	return 0;
}

/**
 * @brief Read a byte from MPU-6050 through I2C 读一个字节数据
 * 以复合模式读取数据
 * @param reg parameter is a register of MPU-6050
 * @param res the data read will be stored in this parameter
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Read_Byte(uint8_t reg, uint8_t *res)
{

	esp_err_t error;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	// 地址广播
	error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, 1);
	if (error != ESP_OK)
		return 1;
	// 写将要读取的寄存器地址
	error = i2c_master_write_byte(cmd, reg, 1);
	if (error != ESP_OK)
		return 1;
	// 起始信号
	i2c_master_start(cmd);
	// 广播地址，并说明要读数据
	error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, 1);
	if (error != ESP_OK)
		return 1;
	// 读上述寄存器数据
	error = i2c_master_read_byte(cmd, res, I2C_MASTER_LAST_NACK);
	if (error != ESP_OK)
		return 1;
	// 停止信号
	i2c_master_stop(cmd);
	// 发送数据
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);
	return 0;
}

/**
 * @brief Read a buffer from MPU-6050 through I2C
 *
 * @param reg parameter is a register of MPU-6050
 * @param buf parameter is a buf witch will store the data
 * @param len parameter is the length of buf
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Read_Len(uint8_t reg, uint8_t *buf, uint8_t len)
{
	esp_err_t error;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, 1);
	if (error != ESP_OK)
		return 1;

	error = i2c_master_write_byte(cmd, reg, 1);
	if (error != ESP_OK)
		return 1;

	i2c_master_start(cmd);
	error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, 1);
	if (error != ESP_OK)
		return 1;

	error = i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
	if (error != ESP_OK)
		return 1;

	i2c_master_stop(cmd);

	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);
	return 0;
}

/**
 * @brief Set the band of low pass filter
 *
 * @param lps parameter is the band of low pass filter
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data);
}

/**
 * @brief Get the temperature of the MPU-6050
 *
 * @return
 *     - temp is the temperature of the MPU-6050
 *     - 1 is Error
 */
int16_t MPU_Get_Temperature()
{
	uint8_t buf[2];
	int16_t raw;
	float temp;
	if (MPU_Read_Len(MPU_TEMP_OUTH_REG, buf, 2) == 0)
		return 1;
	raw = ((uint16_t)(buf[1] << 8)) | buf[0];
	temp = 36.53 + ((double)raw / 340);
	return temp * 100;
}

/**
 * @brief Get the Gyroscope data of the MPU-6050
 *
 * @param gx parameter is the x axis data of Gyroscope
 * @param gy parameter is the y axis data of Gyroscope
 * @param gz parameter is the z axis data of Gyroscope
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Get_Gyroscope(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_GYRO_XOUTH_REG, buf, 6);
	if (res == 0)
	{
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
}

/**
 * @brief Get the Accelerometer data of the MPU-6050
 *
 * @param ax parameter is the x axis data of Accelerometer
 * @param ay parameter is the y axis data of Accelerometer
 * @param az parameter is the z axis data of Accelerometer
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Get_Accelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_ACCEL_XOUTH_REG, buf, 6);
	if (res == 0)
	{
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
}

/******************************************************************************
 * 函  数：void MPU6050_Read(void)
 * 功  能：读取陀螺仪加速度计的原始数据
 * 参  数：无
 * 返回值：无
 * 备  注：无
 *******************************************************************************/
void MPU6050_Read(void)
{
	MPU6050_ReadMultBytes(MPU_ACCEL_XOUTH_REG, 14, MPU6050_buff); // 查询法读取MPU6050的原始数据
}

/******************************************************************************
 * 函  数：uint8_t MPU6050_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
 * 功  能：MPU6050零偏校准
 * 参  数：value： 	 MPU6050原始数据
 *         offset：	 校准后的零偏值
 *         sensivity：加速度计的灵敏度
 * 返回值：1校准完成 0校准未完成
 * 备  注：无
 *******************************************************************************/
uint8_t MPU6050_OffSet(INT16_XYZ value, INT16_XYZ *offset, uint16_t sensivity)
{
	static int32_t tempgx = 0, tempgy = 0, tempgz = 0;
	static uint16_t cnt_a = 0; // 使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
	if (cnt_a == 0)
	{
		value.X = 0;
		value.Y = 0;
		value.Z = 0;
		tempgx = 0;
		tempgy = 0;
		tempgz = 0;
		cnt_a = 1;
		sensivity = 0;
		offset->X = 0;
		offset->Y = 0;
		offset->Z = 0;
	}
	tempgx += value.X;
	tempgy += value.Y;
	tempgz += value.Z - sensivity; // 加速度计校准 sensivity 等于 MPU6050初始化时设置的灵敏度值（8192LSB/g）;陀螺仪校准 sensivity = 0；
	if (cnt_a == 200)			   // 200个数值求平均
	{
		offset->X = tempgx / cnt_a;
		offset->Y = tempgy / cnt_a;
		offset->Z = tempgz / cnt_a;
		cnt_a = 0;
		return 1;
	}
	cnt_a++;
	return 0;
}

/******************************************************************************
 * 函  数：void MPU6050_DataProcess(void)
 * 功  能：对MPU6050进行去零偏处理
 * 参  数：无
 * 返回值：无
 * 备  注：无
 *******************************************************************************/
void MPU6050_GET_OFFSET_DATA(void)
{
	// 加速度去零偏AD值
	// MPU6050_ACC_RAW.X = ((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]) - ACC_OFFSET_RAW.X;
	// MPU6050_ACC_RAW.Y = ((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]) - ACC_OFFSET_RAW.Y;
	// MPU6050_ACC_RAW.Z = ((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]) - ACC_OFFSET_RAW.Z;

	// 加速度去零偏AD值
	MPU6050_ACC_RAW.X = ((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]);
	MPU6050_ACC_RAW.Y = ((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]);
	MPU6050_ACC_RAW.Z = ((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]);
	// // 陀螺仪去零偏AD值
	MPU6050_GYRO_RAW.X = ((((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9]) - GYRO_OFFSET_RAW.X;
	MPU6050_GYRO_RAW.Y = ((((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11]) - GYRO_OFFSET_RAW.Y;
	MPU6050_GYRO_RAW.Z = ((((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13]) - GYRO_OFFSET_RAW.Z;

	// 加速度去零偏AD值
	// MPU6050_ACC_RAW.X = ((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]) ;
	// MPU6050_ACC_RAW.Y = ((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]) ;
	// MPU6050_ACC_RAW.Z = ((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]) ;
	// // // 陀螺仪去零偏AD值
	// MPU6050_GYRO_RAW.X = ((((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9])   ;
	// MPU6050_GYRO_RAW.Y = ((((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11]) ;
	// MPU6050_GYRO_RAW.Z = ((((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13]) ;

	// printf("raw_GRO:%d,%d,%d,%d,%d,%d\n", ACC_OFFSET_RAW.X, ACC_OFFSET_RAW.Y, ACC_OFFSET_RAW.Z, GYRO_OFFSET_RAW.X, GYRO_OFFSET_RAW.Y, GYRO_OFFSET_RAW.Z);
}

void MPU6050_RAWDATA_GET(void)
{
	ACC_OFFSET_RAW.X = ((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]);
	ACC_OFFSET_RAW.Y = ((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]);
	ACC_OFFSET_RAW.Z = ((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]);
	// // 陀螺仪去零偏AD值
	GYRO_OFFSET_RAW.X = ((((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9]);
	GYRO_OFFSET_RAW.Y = ((((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11]);
	GYRO_OFFSET_RAW.Z = ((((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13]);
	// printf("raw_GRO:%d,%d,%d,%d,%d,%d\n", ACC_OFFSET_RAW.X, ACC_OFFSET_RAW.Y, ACC_OFFSET_RAW.Z, GYRO_OFFSET_RAW.X, GYRO_OFFSET_RAW.Y, GYRO_OFFSET_RAW.Z);
}

/*加速度计不能简单的取平均值校准*/
void MPU_OFFSET_GRO(void)
{
	for (int i = 0; i < 300; i++)
	{
		MPU6050_Read(); // 触发读取 ，立即返回
		MPU6050_RAWDATA_GET();
		if (i > 100)
		{
			// offset_acc_x += ACC_OFFSET_RAW.X;
			// offset_acc_y += ACC_OFFSET_RAW.Y;
			// offset_acc_z += ACC_OFFSET_RAW.Z;

			offset_gro_x += GYRO_OFFSET_RAW.X;
			offset_gro_y += GYRO_OFFSET_RAW.Y;
			offset_gro_z += GYRO_OFFSET_RAW.Z;
		}
	}
	// ACC_OFFSET_RAW.X = offset_acc_x / 200;
	// ACC_OFFSET_RAW.Y = offset_acc_y / 200;
	// ACC_OFFSET_RAW.Z = offset_acc_z / 200;
	GYRO_OFFSET_RAW.X = offset_gro_x / 200;
	GYRO_OFFSET_RAW.Y = offset_gro_y / 200;
	GYRO_OFFSET_RAW.Z = offset_gro_z / 200;
	// printf("offset done!:%d,%d,%d,%d,%d,%d\n", offset_acc_x, offset_acc_y, offset_acc_z, offset_gro_x, offset_gro_y, offset_gro_z);
	// printf("offset done!:%d,%d,%d,%d,%d,%d\n", ACC_OFFSET_RAW.X, ACC_OFFSET_RAW.Y, ACC_OFFSET_RAW.Z, GYRO_OFFSET_RAW.X, GYRO_OFFSET_RAW.Y, GYRO_OFFSET_RAW.Z);
}
/******************************************************************************
 * 函  数：void mpu6050_scan_task(void *arg)
 * 功  能：更新当前姿态
 * 参  数：无
 * 返回值：无
 * 备  注：无
 *******************************************************************************/
extern FLOAT_XYZ Acc_filt, Gry_filt, Acc_filtold;

alt_bro ALT_BRO;
alt_acc ALT_ACC;
#define LPF_6_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz)*6.28f * (t)))) * ((in) - (out)))
float wave_high, wave_filter = 0;
uint8_t pid_data[96];
// 定时器回调函数
// void ATimerCallback3(TimerHandle_t xTimer) // 绿灯表示wifi连接，蓝灯表示信号连接
// {
// 	wave_high = recv_data();
// 	LPF_6_(20, 0.001, wave_high, wave_filter);
// }
void mpu6050_scan_task(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = 1; // 间隔
	float temp = 0, pres = 0, hum = 0, alt_3, asl = 0;
	TimerHandle_t xTimer3; // 定时器句柄

	int ifwork, ifwork2;
	float accz, z_acc_bais, va;
	

	float battery;

	static uint8_t cnt = 0;

	I2C_Init();
	ifwork = MPU_Init();
	SPL06Init();
	wave_hight_init();
	reset_bro();
	OFFSET_FLAG = get_offset_start();

	// for (uint16_t i = 0; i < 5000; i++)
	// {
	// 	Prepare_Data();
	// 	IMUupdate(&Gyr_rad, &Acc_filt, &Att_Angle); // 四元数姿态解算
	// 	if (i > 3999)
	// 	{
	// 		offset_pit = offset_pit + Att_Angle.pit;
	// 		offset_rol = offset_rol + Att_Angle.rol;
	// 		// printf("g:%f,%f\n", Att_Angle.rol,Att_Angle.pit);
	// 	}
	// }

	// offset_pit=offset_pit/1000.0;
	// offset_rol=offset_rol/1000.0;

	// xTimer3 = xTimerCreate("Timer3", pdMS_TO_TICKS(100), pdTRUE, (void *)0, ATimerCallback3);
	// xTimerStart(xTimer3, 0);
	while (1)
	{
		cnt++;
		if (cnt == 5)
		{
			wave_high = recv_data();
			LPF_6_(20, 0.001, wave_high, wave_filter);
			cnt = 0;
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency); // 以绝对频率运行,运行一次
		Prepare_Data();								 // 获取姿态解算所需数据
		IMUupdate(&Gyr_rad, &Acc_filt, &Att_Angle);	 // 四元数姿态解算

		//--------------------------------------这段是定高的时候使用的代码-------------------------------------------------
		// printf("length:%f,%f\n", wave_high, wave_filter);
		// Get_zacc_bias_expt(); // 读取此时的Z轴加速度
		// if (OFFSET_FLAG)
		// {
		// 	Height_Get();					 // 得到气压计的高度
		// 	Height_vcc_Get(0.1);
		// 	Strapdown_INS_High(wave_filter); // 气压计高度融合
		// }
		// else if (!OFFSET_FLAG)
		// {
		// 	reset_bro();
		// }
		//---------------------------------------------------------------------------------------------------------------

		//-------------------------------------------手动模式PID调节--------------------------------------------------------------
		Control(1);
		//-------------------------------------------定高模式PID调节--------------------------------------------------------------
		// PID_Z();

		printf("x:%f,%f,%f,%f,%f,%f\n", Att_Angle.rol, Att_Angle.pit, Att_Angle.yaw, Gyr_rad.X, Gyr_rad.Y, Gyr_rad.Z);
	}
}

void pid_chage_task(void *arg)
{
	DebugUartInit();
	uint8_t pidflag = 0;
	while (1)
	{
		// 调参
		receive_pid_data(pid_data, &pidflag);
		if (pidflag)
		{
			pid_change(pid_data);
		}
		vTaskDelay(1 / portTICK_RATE_MS);
	}
}
