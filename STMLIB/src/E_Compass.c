/* Module interract with E compass sensor LSM303DLHC */
#include "E_Compass.h"
/*-----Variables-------------------*/
GPIO_InitTypeDef 				Acc_GPIO_Struct;
I2C_InitTypeDef					Acc_I2C_Struct;
/* ------- I2C configuration (datasheet about connecting stm32 vs peripherals*/
void I2C_ECompass_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	/*----Config I2C pin-----*/
	Acc_GPIO_Struct.GPIO_Pin   							= GPIO_Pin_6|GPIO_Pin_9;
	Acc_GPIO_Struct.GPIO_Mode								= GPIO_Mode_AF;
	Acc_GPIO_Struct.GPIO_OType							= GPIO_OType_OD;
	Acc_GPIO_Struct.GPIO_PuPd								= GPIO_PuPd_UP;
	Acc_GPIO_Struct.GPIO_Speed							= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&Acc_GPIO_Struct);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
	/*----I2C config-----*/
	Acc_I2C_Struct.I2C_Mode                 = I2C_Mode_I2C;
	Acc_I2C_Struct.I2C_DutyCycle						= I2C_DutyCycle_2;
	Acc_I2C_Struct.I2C_OwnAddress1					= 0x00;
	Acc_I2C_Struct.I2C_Ack									= I2C_Ack_Enable;
	Acc_I2C_Struct.I2C_ClockSpeed						= 100000;    //100KHz
	Acc_I2C_Struct.I2C_AcknowledgedAddress	= I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1,&Acc_I2C_Struct);
	I2C_Cmd(I2C1,ENABLE);
}
/*------- I2C start transmit ------*/
CheckEVStatus I2C_Start_Transmit(uint8_t addr)
{
	uint16_t time_out;
	
	I2C_GenerateSTART(I2C1,ENABLE);
	
	time_out        = Sensor_TimeOut;
	//Check on EV5
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
	{
		if ((time_out--) == 0)
		{return Not_OK;}
	}
	//Send slave address
	I2C_Send7bitAddress(I2C1,addr,I2C_Direction_Transmitter);
	
	//Check on EV6
	time_out  			= Sensor_TimeOut;
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((time_out--) == 0)
		{return Not_OK;}
	}
	return OK;
}

/*------------I2C Start receive----------------*/
CheckEVStatus I2C_Start_Receive(uint8_t addr)
{
	uint16_t time_out;
	
	I2C_GenerateSTART(I2C1,ENABLE);
	
	//Check on EV5
	time_out = Sensor_TimeOut;
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
	{
		if (time_out-- == 0)
			return Not_OK;
	}
	
	I2C_Send7bitAddress(I2C1,addr,I2C_Direction_Receiver);
	
	//Check on EV6
	time_out = Sensor_TimeOut;
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if (time_out-- == 0)
			return Not_OK;
	}
	return OK;
}
/*-------- Write To Sensor Ecompass --------------*/
CheckEVStatus WriteToSensor(char key, uint8_t register_addr, uint8_t data)
{
	//Check bus busy or not
	uint16_t time_out;
	uint8_t  add_write;
	if(key == 'A')
	{
		add_write = Accelerometer_Write;
	}
	else if (key == 'M')
	{
		add_write = Magnetometer_Write;
	}
	else return Not_OK;
	time_out = Sensor_TimeOut;
	while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY))
	{
		if (time_out-- == 0)
			return Not_OK;
	}
	if (I2C_Start_Transmit(add_write))
	{
		I2C_SendData(I2C1,register_addr);
		time_out = Sensor_TimeOut;
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			if (time_out-- == 0)
				return Not_OK;
		}
		I2C_SendData(I2C1,data);
		time_out = Sensor_TimeOut;
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			if (time_out-- == 0)
				return Not_OK;
		}
		I2C_GenerateSTOP(I2C1,ENABLE);
	}
	else return Not_OK;
	return OK;
}
/*---------- Read from Sensor Ecompass ---------------*/
CheckEVStatus ReadFromSensor(char key, uint8_t register_addr, uint8_t *data)
{
	uint16_t time_out;
	uint8_t add_read,add_write;
	if(key == 'A')
	{
		add_read = Accelerometer_Read;
		add_write = Accelerometer_Write;
	}
	else if (key == 'M')
	{
		add_read = Magnetometer_Read;
		add_write = Magnetometer_Write;
	}
	else return Not_OK;
	time_out = Sensor_TimeOut;
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY))
	{
		if(time_out-- == 0)
			return Not_OK;
	}
	if(I2C_Start_Transmit(add_write))
	{
		I2C_SendData(I2C1,register_addr);
		time_out = Sensor_TimeOut;
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			if(time_out-- == 0)
				return Not_OK;
		}
		if(I2C_Start_Receive(add_read))
		{
			I2C_AcknowledgeConfig(I2C1,ENABLE);
			time_out = Sensor_TimeOut;
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
			{
				if(time_out-- == 0)
					return Not_OK;
			}
			*data = I2C_ReceiveData(I2C1);
		}
		else return Not_OK;
	}
	else return Not_OK;
	return OK;
}
/*----------Read 3 axis value from accelerometer-----------------*/
CheckEVStatus Read3AxisAccelerometer(double *pBuffer)
{
	uint16_t time_out;
	uint8_t  TempBuffer[6];
	int16_t  Temp[3];
	uint8_t  start_addr = 0x28;
	for (int i = 0; i < 6; i++)
	{
		time_out = Sensor_TimeOut;
		while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY))
		{
			if(time_out-- == 0)
				return Not_OK;
		}
		if (I2C_Start_Transmit(Accelerometer_Write))
		{
			I2C_SendData(I2C1,start_addr + i);
			time_out = Sensor_TimeOut;
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				if (time_out-- == 0)
					return Not_OK;
			}
			if (I2C_Start_Receive(Accelerometer_Read))
			{
				I2C_AcknowledgeConfig(I2C1,ENABLE);
				time_out = Sensor_TimeOut;
				while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
				{
					if (time_out-- == 0)
						return Not_OK;
				}
				TempBuffer[i] = I2C_ReceiveData(I2C1);
			}
			else return Not_OK;
		}
		else return Not_OK;
		I2C_AcknowledgeConfig(I2C1,DISABLE);
		I2C_GenerateSTOP(I2C1,ENABLE);
	}
	Temp[0] = TempBuffer[1] * 256 + TempBuffer[0];
	pBuffer[0] = (float)(4 * Temp[0]) / (float)65535;
	Temp[1] = TempBuffer[3] * 256 + TempBuffer[2];
	pBuffer[1] = (float)(4 * Temp[1]) / (float)65535;
	Temp[2] = TempBuffer[5] * 256 + TempBuffer[4];
	pBuffer[2] = (float)(4 * Temp[2]) / (float)65535;
	return OK;
}
/* Algorithm for 1 - point tumble calobration */
void Acc_Calibration(double accX, double accY, double accZ, double *offset)
{
	offset[0] = accX;
	offset[1] = accY;
	offset[2] = accZ - 1;
}
/** @brief  : Functions calculate 3 axis angele
**  @agr    : 3 axis values, and buffer values
**	@Retval : None
**/
void Acc_AngleCalculate(double ax, double ay, double az, double *result)
{
	//Enter your code here
}
/*--------------Magnetometer Functions------------------*/
CheckEVStatus	Read3AxisMagnetometer(double *pBuffer)
{
	uint16_t time_out;
	uint8_t  TempBuffer[6];
	int16_t  Temp[3];
	uint8_t  start_addr = 0x03;
	for (int i = 0; i < 6; i++)
	{
		time_out = Sensor_TimeOut;
		while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY))
		{
			if(time_out-- == 0)
				return Not_OK;
		}
		if (I2C_Start_Transmit(Magnetometer_Write))
		{
			I2C_SendData(I2C1,start_addr + i);
			time_out = Sensor_TimeOut;
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				if (time_out-- == 0)
					return Not_OK;
			}
			if (I2C_Start_Receive(Magnetometer_Read))
			{
				I2C_AcknowledgeConfig(I2C1,ENABLE);
				time_out = Sensor_TimeOut;
				while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
				{
					if (time_out-- == 0)
						return Not_OK;
				}
				TempBuffer[i] = I2C_ReceiveData(I2C1);
			}
			else return Not_OK;
		}
		else return Not_OK;
		I2C_AcknowledgeConfig(I2C1,DISABLE);
		I2C_GenerateSTOP(I2C1,ENABLE);
	}
	for (int i = 0; i < 3; i++)
	{
		Temp[i] = TempBuffer[i*2] * 256 + TempBuffer[i*2 + 1];
		pBuffer[i] = ((double)Temp[i] / 1100);
	}
	return OK;
}

//D = arctan(yGaussData/xGaussData)*(180/p)
double GetAngle(void)
{
	double result;
	double Mxyz[3], Mx, My;
	Read3AxisMagnetometer(Mxyz);
	Mx = Mxyz[0];
	My = Mxyz[2];
	if(Mx != 0)
	{
	result = atan(My/Mx) * ((double)180 / 3.1415926535897931);
	if(result > 360) result -= 360;
	else if (result < 0) result += 360;
	}
	else result = 0;
	return result;
}

double ScaleAngle(double D)
{
	double result;
	// angle : -180 - 0 - 180
	if(D > 180)
		result = D - 360;
	else
		result = D;
	return result;
}

double GetAngle_LSM303DLHC(void)
{
	double Phi, Gz2, Theta, By2, Bz2, Bx3, Yaw, Mxzy[3], Axyz[3];
	Read3AxisAccelerometer(Axyz);
	Read3AxisMagnetometer(Mxzy);
	Phi = atan2(Axyz[1], Axyz[2]);
	Gz2 = Axyz[1] * sin(Phi) + Axyz[2] * cos(Phi);
	Theta = atan(-Axyz[0] / Gz2);
	By2 = Mxzy[1] * sin(Phi) - Mxzy[2] * cos(Phi);
	Bz2 = Mxzy[2] * sin(Phi) + Mxzy[1] * cos(Phi);
	Bx3 = Mxzy[0] * cos(Theta) + Bz2 * sin(Theta);
	Yaw = (atan2(By2,Bx3)*180)/3.14159;
	return Yaw;
}
char* CalculateDirection(double Angle)
{
	char *c;
	if((Angle > 337.25) || (Angle < 22.5)) 
	{
		c[0] = 'N';
		c[1] = 'F';
	}
	else if ((Angle > 292.5) && (Angle <= 337.25))
	{
		c[0] = 'N';
		c[1] = 'W';
	}
	else if ((Angle > 247.5) && (Angle <= 292.5))
	{
		c[0] = 'W';
		c[1] = 'F';
	}
	else if ((Angle > 202.5) && (Angle <= 247.5))
	{
		c[0] = 'S';
		c[1] = 'W';
	}
	else if ((Angle > 157.5) && (Angle <= 202.5))
	{
		c[0] = 'S';
		c[1] = 'F';
	}
	else if ((Angle > 112.5) && (Angle < 157.5))
	{
		c[0] = 'S';
		c[1] = 'E';
	}
	else if ((Angle > 67.5) && (Angle <= 112.5))
	{
		c[0] = 'E';
		c[1] = 'F';
	}
	else
	{
		c[0] = 'N';
		c[1] = 'E';
	}
  return c;
}
/*-------------- ECompass IC IMU (Shield protection unit) --------------------*/

/*--------------ECompass sensor Init-----------------*/
void ECompass_Config()
{
	// ----Normal/ Low power mode (50Hz) (Ctrl Reg 1: 20h)
	I2C_ECompass_Config();
	WriteToSensor('A',LSM303DLHC_Ctrl_Reg_1,LSM303DLHC_Mode_Normal_50Hz_3Axis);
	WriteToSensor('M',LSM303DLHC_CRB_REG_M,LSM303DLHC_Gain_1);
	WriteToSensor('M',LSM303DLHC_MR_REG_M,0x00);
}

/*-------------- SRF05 ----------------*/
/** @brief  : Defuzzification Max Prod sugeno
**  @agr    : Input value, name of the symbols value
**  @retval : Output value
**/
void SRF05_Config()
{
	
}























