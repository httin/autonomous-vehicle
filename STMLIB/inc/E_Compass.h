#include   "stm32f4xx.h"
#include   <math.h>
/* Module to interract with Ecompass sensor (LSM303DLHC) included in stm32f4 
discovery board. 
-- LSM303DLHC datasheet
-- 2 sensor combine into E compass module: Accelerometer and Magnetic sensor

++ LSM303DLHC: I2C interface PB6 - I2C1_SDA, PB9 - I2C1_SCL, read datasheet I2C

*/

/* Define */
#define   		Xoffset																	 0.027344167232513428
#define				Yoffset																   -0.0312504768371582
#define				Zoffset																	 1.058609962463379-1
/* ---------- Define for Accelerometer sensor 3 axis ------------------*/
#define 			Accelerometer_Read 											 0x33
#define				Accelerometer_Write		 									 0x32
#define				Magnetometer_Read												 0x3D
#define				Magnetometer_Write											 0x3C
#define       LSM303DLHC_Ctrl_Reg_1                    0x20
#define       LSM303DLHC_Mode_Normal_50Hz_3Axis        0x47
#define       LSM303DLHC_CRB_REG_M										 0x01
#define 			LSM303DLHC_MR_REG_M											 0x02
#define 			LSM303DLHC_Gain_1												 0x20
#define 			Sensor_TimeOut										 			 20000
/* ---------- Define for SRF05                   ----------------------*/

/* Define types */
typedef enum{
	Not_OK = 0,
	OK,
}CheckEVStatus;

/* Export variables */

/* Export functions */
void 					ECompass_Config(void);
CheckEVStatus WriteToSensor(char key, uint8_t register_addr, uint8_t data);
CheckEVStatus ReadFromSensor(char key, uint8_t register_addr, uint8_t *data);
/* Accelerometer functions */
void					Acc_Calibration(double accX, double accY, double accZ, double *offset);
CheckEVStatus Read3AxisAccelerometer(double *pBuffer);
void 					Acc_AngleCalculate(double ax, double ay, double az, double *result);
/* Magnetometer functions */
void 					Mag_Calibration(double magX, double macY, double macZ, double *offset);
CheckEVStatus	Read3AxisMagnetometer(double *pBuffer);
double 				GetAngle(void);
double 				ScaleAngle(double D);
char*					CalculateDirection(double Angle);
double 				GetAngle_LSM303DLHC(void);
/* Thay Hao Sensor */

















