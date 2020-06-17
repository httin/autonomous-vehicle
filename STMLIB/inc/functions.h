#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <math.h>
#include "stm32f4xx.h"
#include "USART_DMA_Config.h"

#define    SYSTICK_INTERRUPT_1MS 1000
#define    SAMPLE_TIME_50MS 50
#define    SEND_TIME_1S 1000

#define    MESSAGE_ROW 15
#define    MESSAGE_COL 20
#define    MAX_LORA_BUFFERSIZE 58
/********************************************************************
 *							Define Enum 							*
 ********************************************************************/
typedef enum
{
	Check_NOK = 0,
	Check_OK = 1,
}enum_Status;

typedef enum{
	Veh_NoneError = 0,

	Veh_ReadGxGLLMessage_Err,		// GPS
	Veh_ReadGxGGAMessage_Err,		// GPS
	Veh_GxGLLCheckSum_Err,			// GPS: GxGLL CheckSum Error
	Veh_GxGGACheckSum_Err,			// GPS: GxGGA Checksum Error
	Veh_InvalidGxGLLMessage_Err,	// GPS

	LORA_WrongCheckSum, // Lora: message was received from PC is wrong
	IMU_WrongMessage,  // IMU: Receive wrong message format, see IMU_GetValueFromMessage()
}enum_Error;

/****** COMMAND FROM PC ******/
typedef	enum{			// see MsgToCmd()
	None = 0,
	Vehicle_Config,		// 1.
	Sample_Time,		// 2. Update sample time command
	SendData_Time,		// 3. Update send data time command
	IMU_Config,			// 4.
	Soft_Reset,			// 5.
	Manual_Config,		// 6.
	Auto_Config,		// 7.
	Path_Plan,			// 8.
	Flash_Save,			// 9.
	KeyBoard_Control,	// 10. 
}enum_Command;

typedef enum{
	Stanley_Controller = 1,
	Pursuit_Controller,
}enum_Controller;

typedef enum{
	None_Mode = 0,
	Auto_Mode,
	Manual_Mode,
	Calib_Mode,
	KeyBoard_Mode,
	Soft_Reset_Mode,
}enum_Mode;

typedef enum{
	Invalid        = 0,
	Mode_2D_3D     = 1,
	DGNSS          = 2,
	Fixed_RTK      = 4,
	Float_RTK      = 5,
	Dead_Reckoning = 6,
}enum_GPS_Quality;

/********************************************************************
 *							Define Struct 							*
 ********************************************************************/
typedef struct Status{
	enum_Status 		IMU_FirstSetAngle;			// 0. IMU first received angle after turning on
	enum_Status			Veh_Sample_Time;			// 1. Vehicle sample time finished
	enum_Status			Veh_Send_Data;				// 2. Vehicle send data time finished
	enum_Status			GPS_Coordinate_Received;	// 3. 
	enum_Status			GPS_ValidGPS;				// 4.
	enum_Status			Veh_SendData_Flag;			// 5. Flag to ENABLE send data from vehicle, set/unset by F7/F8
	enum_Status			Veh_Calib_Flag;				// 6.
	enum_Status			Veh_Timer_Finish;			// 8. Timer 5 Stop 
	enum_Status			Veh_Timer_Start;			// 9. Timer 5 Start 
	enum_Status			IMU_Calib_Finish;			// 10. Calibration IMU 
	enum_Status			Veh_Auto_Flag;				// 12.
	enum_Status			GPS_Start_Receive_PathCor;	// 13. Starting receive map coordinate from C#
	enum_Status			GPS_SelfUpdatePosition_Flag;// 14.
	enum_Status			GPS_FirstGetPosition;		// 15.
} Status;

typedef struct Error{
	uint8_t			Error_Buffer[20];	/* enum_Error Code */
	uint8_t			Error_Index; 		/* Current Index of Error_Buffer */
} Error;

typedef struct Time{
	double      T; // = (ms to sample) * (1 / SYSTICK_INTERRUPT) (s)
	uint32_t    sample_time; // Thoi gian lay mau cua VXL (ms)
	uint32_t    sample_count; 
	uint32_t    send_time;
	uint32_t    send_count;
} Time;

typedef	struct  DCMotor{
	/* PID parameters */
	double		Kp;
	double 		Ki;
	double		Kd;
	/* Input and Output of PID controller */
	double		Set_Vel;
	double 		Current_Vel;	
	double		*SampleTime;
	double		Pre_PID;
	double 		Pre_Error;
	double 		Pre2_Error;
	double		PID_Out;
	/* Encoder parameters */
	uint16_t  	Enc;
	uint16_t 	PreEnc;
	uint8_t		OverFlow;
	uint8_t		Change_State; // 0 or 1
} DCMotor;

/* Triangle function */
typedef struct trimf{
	double a1;
	double a2;
	double a3;
} trimf;

/* Trapezoid function */
typedef struct trapf{
	double h1;
	double h2;
	double h3;
	double h4;
} trapf;

typedef struct IMU{
	/* Current Angle and Set Angle */
	double      Angle;
	double      Set_Angle;
	double      Pre_Angle;
	/* Fuzzy input and output */
	double      Fuzzy_Out;
	double      Fuzzy_Error;
	double      Fuzzy_Error_dot;
	/* Variables Ke, Kedot and Ku */
	double      Ke;
	double      Kedot;
	double      Ku;
} IMU;

typedef struct GPS{
	/* Robot state */
	double              CorX;
	double              CorY;
	double              Pre_CorX;
	double              Pre_CorY;
	double              dx;
	double              dy;
	int                 NewDataAvailable;
	int                 Times;
	IMU                 *Angle;
	int                 P_Yaw_Index;
	double              efa;
	/* Stanley control variables */
	double              Thetae;
	double              Thetad;
	double              Delta_Angle;
	double              K;
	double              Step;
	double              Robot_Velocity; // (Vr + Vl) / 2
	double              dmin;
	/* Goal radius reached */
	enum_Status         Goal_Flag;
	/* GPS NEO M8P input coordinates */
	double              Latitude;
	double              Longitude;
	int                 NbOfWayPoints;
	enum_GPS_Quality    GPS_Quality;
	int                 NbOfP;
	int                 Cor_Index;
	/* Buffer read and write data */
	double              Path_X[20];
	double              Path_Y[20];
#define MAX_NUM_COORDINATE 1000
	double              P_X[MAX_NUM_COORDINATE];  
	double              P_Y[MAX_NUM_COORDINATE];  
	double              P_Yaw[MAX_NUM_COORDINATE];
	/* Error GPS code */
	enum_Error          GPS_Error;
} GPS;

typedef	struct Vehicle
{
	double             Max_Velocity;
	double             Manual_Velocity;
	double             Manual_Angle;
	double             Sensor_Angle;
	enum_Mode          Mode;
	enum_Error         Veh_Error;
	enum_Controller    Controller;
	int	               SendData_Ind;
	/* Calibration variables */
	uint16_t           Distance;
	uint8_t	           TotalDistance;
	char               ManualCtrlKey;
} Vehicle;

typedef	struct Message
{
	char    Message[MESSAGE_ROW][MESSAGE_COL];
} Message;

typedef struct FlashMemory{
	uint32_t 	WriteIn32bBuffer[100];
	uint8_t		ReadOutBuffer[500];
	uint8_t		WriteInBuffer[500];
	int			Length;
	char  		Message[MESSAGE_ROW][MESSAGE_COL];
} FlashMemory;

#define					pi							(double)3.14159265358979
#define  				K1 										1/(2*pi)
#define					K2										4/pi
#define					K3										1
#define					Wheel_Radius 							0.085
#define					IMU_AngleIndex							17
#define					FLASH_ProgramType_Byte					VoltageRange_1
#define					FLASH_ProgramType_HalfWord				VoltageRange_2
#define					FLASH_ProgramType_Word					VoltageRange_3
#define					FLASH_ProgramType_DoubleWord			VoltageRange_4
#define					FLASH_PIDPara_BaseAddr 					0x08060000	// (4 KBytes) (0x08060000 - 0x08060FFF)
#define					FLASH_FuzPara_BaseAddr					0x08061000	// (4 Kbytes) (0x08061000 - 0x08061FFF)
#define					FLASH_GPSPara_BaseAddr					0x08040000	// (128 KBytes) 
/* Control Led Macros */
#define LED_RED_PIN      GPIO_Pin_14
#define LED_GREEN_PIN    GPIO_Pin_12
#define LED_BLUE_PIN     GPIO_Pin_15
#define LED_ORANGE_PIN   GPIO_Pin_13

#define LED_ON(LED_PIN)	 GPIOD->BSRRL = LED_PIN
#define LED_OFF(LED_PIN) GPIOD->BSRRH = LED_PIN
#define LED_ON_ALL() GPIOD->BSRRL = LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN|LED_ORANGE_PIN
#define LED_OFF_ALL() GPIOD->BSRRH = LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN|LED_ORANGE_PIN
#define LED_TOGGLE(LED_PIN) GPIOD->ODR ^= LED_PIN
/* Export variables */
extern Error           Veh_Error;
extern Time            Timer;
extern Status          VehStt;
extern FlashMemory     Flash;
extern DCMotor         M1, M2;
extern IMU             Mag;
extern GPS             GPS_NEO;
extern Message         U2, U6; // USART2 & USART6 message
extern Vehicle         Veh;
extern double          NB, NM, NS, ZE, PS, PM, PB;
extern trimf           In1_NS, In1_ZE, In1_PS, In2_ZE;
extern trapf           In1_NB, In1_PB, In2_NE, In2_PO;
extern char	           TempBuffer[2][30];
/*--------Export Function------------------------- */
uint8_t 				ToChar(double value, uint8_t *pBuffer,int NbAfDot); // Convert double value to char array
uint8_t					ToHex(uint8_t input);
double					MPS2RPM(double vel);
double 					Pi_To_Pi(double angle);
int						LengthOfLine(uint8_t *inputmessage);
double 					Degree_To_Degree(double angle);
void					Convert_Double_Array(double *pInputArray, int n);
/*------------ Error update -------------*/
void					Error_AppendError(Error *perror, enum_Error err);
/*------------ Vehicle Status update -------------*/
void					Status_ParametersInit(Status *pstt);
/*------------ Timer Function --------------------*/
void					Time_ParametersInit(Time *pTime, uint32_t sample_time_init, uint32_t send_time_init);
void					Time_SampleTimeUpdate(Time *pTime, uint32_t sample_time_update);
#define 				Time_SendTimeUpdate(pTime, send_time_in_ms)	(pTime)->send_time = (send_time_in_ms); 
/*------------ Vehicle status functions ----------*/
void					Veh_ParametersInit(Vehicle *pveh);
void					Veh_UpdateVehicleFromKey(Vehicle *pveh);
void					Veh_CheckStateChange(DCMotor *ipid, uint8_t State);
#define 				Veh_UpdateMaxVelocity(pveh, MaxVelocity)	(pveh)->Max_Velocity = (MaxVelocity);
enum_Error              Veh_SplitMsg(uint8_t *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL]);
enum_Command            Veh_MsgToCmd(char *);
/*------------ PID Function ----------------------*/
void 					PID_SavePIDParaToFlash(FlashMemory *pflash, DCMotor *M1, DCMotor *M2);
void 					PID_Compute(DCMotor *ipid);
void 					PID_ParametersInitial(DCMotor *ipid);
#define 				PID_UpdateSetVel(DCMotor, SetVal)	(DCMotor)->Set_Vel = (SetVal) 
void 					PID_ParametersUpdate(DCMotor *ipid, double Kp, double Ki, double Kd);
void 					PID_ResetEncoder(DCMotor *ipid);
void 					PID_ResetPID(DCMotor *ipid);
/* -------Send and Receive data function------------ */
void                    GetMessageInfo(char *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL], char character);
double                  GetValueFromString(char *value);
uint8_t	                LRCCalculate(uint8_t *pBuffer, int length);
enum_Status             IsCorrectMessage(uint8_t *inputmessage, int length, uint8_t byte1, uint8_t byte2);
enum_Status             StringHeaderCompare(char *s1, char header[]);
int                     FeedBack(uint8_t *outputmessage, char inputstring[20]);
/*--------Stanley functions and GPS --------------*/
void                    GPS_ParametersInit(GPS *pgps);
void                    GPS_StanleyControl(GPS *pgps, double SampleTime, double M1Velocity, double M2Velocity);
void                    GPS_PursuitControl(GPS *pgps, double SampleTime, double M1Velocity, double M2Velocity);
double                  GPS_DMS_To_DD(double LL);
double	                GPS_StringToLat(char *inputmessage);
double                  GPS_StringToLng(char *inputmessage);
void                    GPS_LatLonToUTM(GPS *pgps);  //Get 2 values of lat-lon and update UTM coordiante to Corx and Cory
void                    GPS_ClearPathBuffer(GPS *pgps);
void                    GPS_ClearPathCorBuffer(GPS *pgps);
void                    GPS_UpdatePathYaw(GPS *pgps);
void                    GPS_SavePathCoordinateToFlash(GPS *pgps, FlashMemory *pflash);
void                    GPS_UpdateCoordinateXY(GPS *pgps, double Cor_X, double Cor_Y);
void                    GPS_PathPlanning(GPS *pgps, float Step);
enum_Status	            GPS_HeaderCompare(uint8_t *s1, char Header[5]);
enum_Error              GPS_GetLLQMessage(GPS *pgps, uint8_t *inputmessage,char result[MESSAGE_ROW][MESSAGE_COL]);
/*--------Fuzzy control-------------------*/
void                    Fuzzy_ParametersInit(void);
double                  Trapf(trapf *ptrapf, double x);
double                  Trimf(trimf *ptrimf, double x);
void                    Trimf_Update(trimf *ptrimf, double a1, double a2, double a3);
void                    Trapf_Update(trapf *ptrapf, double a1, double a2, double a3, double a4);
void                    Defuzzification_Max_Min(IMU *pimu);

/*--------IMU functions ---------*/
void                    IMU_ParametesInit(IMU *pimu);
void                    IMU_UpdateSetAngle(IMU *pimu, double ComAngle);
void                    IMU_UpdatePreAngle(IMU *pimu);
void                    IMU_UpdateFuzzyInput(IMU *pimu, double *pSampleTime);
void                    IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku);
enum_Error              IMU_GetValueFromMessage(IMU *pimu, uint8_t *inputmessage);

/*-------- Flash Memory Embedded functions --------*/
void                    WriteToFlash(FlashMemory *pflash, uint32_t FLASH_Sector, uint32_t FLASH_BaseAddr);
void                    ReadFromFlash(FlashMemory *pflash, uint32_t FLASH_BaseAddr);
void                    EraseMemory(uint32_t Flash_Sector);

#endif
