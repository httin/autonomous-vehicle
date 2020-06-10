#include "functions.h"

/* Global variables */

/*----- Stanley Variables -----*/
/*----- Robot Parameter -------*/
Error	        Veh_Error;
Time            Timer;
Status          VehStt;
FlashMemory     Flash;
DCMotor         M1, M2, Ang;
IMU	            Mag;
GPS             GPS_NEO;
Message         U2, U6;
Vehicle	        Veh;
double          NB,NM,NS,ZE,PS,PM,PB;
trimf           In1_NS,In1_ZE,In1_PS,In2_ZE;
trapf           In1_NB,In1_PB,In2_NE,In2_PO;
char            TempBuffer[2][30];
/*----- Error functions -----------------------------------*/
void	Error_AppendError(Error *perror, enum_Error err)
{
	perror->Error_Buffer[perror->Error_Index++] = err;
}
/*----- Init, and function to config vehicle status -------*/
void	Status_ParametersInit(Status *pStatus)
{
	/* VehStt is initialized as a global variable, so all members were 0 */
	pStatus->Veh_SendData_Flag = Check_NOK;
}

/** @brief  : Function compute PID value 
**	@agr    : void
**	@retval : None
**/
void PID_Compute(DCMotor *ipid)
{
	ipid->SampleTime = &Timer.T;
	ipid->PID_Out = ipid->Pre_PID + ipid->Kp * (ipid->Set_Vel - ipid->Current_Vel) + 0.5 * ipid->Ki * *(ipid->SampleTime) * ((ipid->Set_Vel - ipid->Current_Vel) + ipid->Pre_Error) + (ipid->Kd / *(ipid->SampleTime)) * ((ipid->Set_Vel - ipid->Current_Vel) - 2 * ipid->Pre_Error + ipid->Pre2_Error);
	ipid->Pre2_Error = ipid->Pre_Error;
	ipid->Pre_Error = ipid->Set_Vel - ipid->Current_Vel;
	if(ipid->PID_Out < 0)
		ipid->PID_Out = (double)0;
	if(ipid->PID_Out > 100)
		ipid->PID_Out = (double)100;
	ipid->Pre_PID = ipid->PID_Out;
}

/** @brief  : First initial PID parameters
**	@agr    : void
**	@retval : None
**/
void PID_ParametersInitial(DCMotor *ipid)
{
	/* M1, M2 were initialized as a global variable */
	ipid->Change_State = 1;
}

/** @brief  : PID update parameters function
**	@agr    : void
**	@retval : None
**/
void PID_ParametersUpdate(DCMotor *ipid, double Kp, double Ki, double Kd)
{
	ipid->Kp = Kp;
	ipid->Ki = Ki;
	ipid->Kd = Kd;
}

/** @brief  : PID reset encoder
**	@retval : None
**/
void	PID_ResetEncoder(DCMotor *ipid)
{
	ipid->PreEnc = ipid->Enc;
	ipid->OverFlow = 1;
}

/** @brief  : PID reset PID
**	@retval : None
**/
void	PID_ResetPID(DCMotor *ipid)
{
	ipid->Pre_PID = 0;
	ipid->Pre_Error = 0;
	ipid->Pre2_Error = 0;
}

/** @brief  : PID Save parameters to interflash memory
**	@retval : None
**/
void	PID_SavePIDParaToFlash(FlashMemory *pflash, DCMotor *M1, DCMotor *M2)
{
	pflash->Length = 0;
	pflash->Length += ToChar(M1->Kp,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M1->Ki,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M1->Kd,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M2->Kp,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M2->Ki,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M2->Kd,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = 0x0D;
	pflash->WriteInBuffer[pflash->Length++] = 0x0A;
	EraseMemory(FLASH_Sector_7);
	WriteToFlash(pflash,FLASH_Sector_7,FLASH_PIDPara_BaseAddr);
}

/** @brief  : Calculate length of a line (end with 0x0D,0x0A "\r\n")
**  @agr    : Input buffer
**  @retval : Length
**/
int	LengthOfLine(uint8_t *inputmessage)
{
	int length = 0;
	while(	(inputmessage[length] != 0) &&
			(inputmessage[length] != 0x0D) && 
			(inputmessage[length + 1] != 0x0A))
	{
		length++;
	}
	return length;
}

/* ----------------------- Timer functions -----------------------------------*/
void	Time_ParametersInit(Time *ptime, uint32_t sample_time_init, uint32_t send_time_init)
{
	ptime->sample_time 	= sample_time_init;
	ptime->sample_count = 0;
	ptime->send_time 	= send_time_init;
	ptime->send_count 	= 0;
	ptime->T = ptime->sample_time * pow(10, -3); 
}

void	Time_SampleTimeUpdate(Time *ptime, uint32_t sample_time_update)
{
	ptime->sample_time = sample_time_update;
	ptime->T = ptime->sample_time * pow(10, -3);
}

/*------------------------ Vehicle Status Function ----------------------------*/
void	Veh_ParametersInit(Vehicle *pveh)
{
	pveh->Max_Velocity = MPS2RPM(1);
	pveh->Mode = KeyBoard_Mode;
	pveh->Veh_Error = Veh_NoneError;
	pveh->Controller = Stanley_Controller;
}

void	Veh_UpdateVehicleFromKey(Vehicle *pveh)
{
	if(pveh->ManualCtrlKey == 'W')
	{
		IMU_UpdateSetAngle(&Mag,0);
		pveh->Manual_Velocity += 0.2 * pveh->Max_Velocity;
	}
	else if(pveh->ManualCtrlKey == 'S')
	{
		IMU_UpdateSetAngle(&Mag,0);
		pveh->Manual_Velocity -= 0.2 * pveh->Max_Velocity;
	}
	else if(pveh->ManualCtrlKey == 'D')
	{
		pveh->Manual_Angle += 30;
		if(pveh->Manual_Angle > 180) pveh->Manual_Angle -= 360;
		IMU_UpdateSetAngle(&Mag,pveh->Manual_Angle);
	}
	else if(pveh->ManualCtrlKey == 'A')
	{
		pveh->Manual_Angle -= 30;
		if(pveh->Manual_Angle < -180) pveh->Manual_Angle += 360;
		IMU_UpdateSetAngle(&Mag,pveh->Manual_Angle);
	}
	if(pveh->Manual_Velocity > pveh->Max_Velocity) pveh->Manual_Velocity = pveh->Max_Velocity;
	else if(pveh->Manual_Velocity < 0) pveh->Manual_Velocity = 0;
	pveh->ManualCtrlKey = 0;
}

enum_Error	Veh_SplitMsg(uint8_t *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL])
{
	int length = LengthOfLine(inputmessage); // Ex: "$ab,cd\r\n" -> length = 6, "$ab,cd\0" -> length = 6
	if(IsCorrectMessage((uint8_t*)&inputmessage[1], length - 3, inputmessage[length - 2], inputmessage[length - 1]))
	{
		GetMessageInfo((char*)inputmessage, result, ',');
		return Veh_NoneError;
	}
	else
		return LORA_WrongCheckSum;
}

void	Veh_CheckStateChange(DCMotor *ipid, uint8_t State)
{
	if(ipid->Change_State != State)
	{
		PID_ResetPID(ipid);
		ipid->Change_State = State;
	}
}


/* ----------------------- GPS function ---------------------------------------*/

/** @brief  : Seperating each info of message by ','
 ** @agr    : input message, result buffer
 ** @retval : None
 **/
void GetMessageInfo(char *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL], char character)
{
	int col = 0, row = 0, index = 0;

	for(int i = 0; (i < MESSAGE_ROW) && (result[i][0] != 0); ++i)
		for(int j = 0; (j < MESSAGE_COL) && (result[i][j] != 0); ++j)
			result[i][j] = 0;

	while(	(inputmessage[index] != 0x0D) && (inputmessage[index + 1] != 0x0A) 
			&& (inputmessage[index] != 0) && (index < 100))
	{
		if(inputmessage[index] != character)
		{
			result[row][col] = inputmessage[index];
			col++;
		}
		else 
		{
			row++;
			col = 0;
		}
		index++;
	}
}

/** @brief  : Get double value from string or array
 ** @agr    : Input string or array
 ** @retval : Double output value
 **/
double GetValueFromString(char *value)
{
	double p1 = 0,p2 = 0, h = 1, result;
	int row = 0, col = 0, index = 0, len1, len2, sign = 1;
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 30; j++)
		{
			TempBuffer[i][j] = 0;
		}
	}
	if(value[0] == '-') 
	{
		index++;
		sign = -1;
	}
	else if(value[0] == ' ')
	{
		index++;
		sign = 1;
	}
	while(value[index] != 0)
	{
		if(value[index] != '.')
		{
			TempBuffer[row][col] = value[index];
			col++;
		}
		else
		{
			row++;
			len1 = col;
			col = 0;
		}
		index++;
	}
	if(row == 0)
	{
		len1 = col;
		for(int i = len1 - 1; i >= 0; i--)
		{
			p1 += ((uint8_t)TempBuffer[0][i] - 48) * h;
			h *= 10;
		}
		p2 = 0;
	}
	else
	{
		for(int i = len1 - 1; i >= 0; i--)
		{
			p1 += ((uint8_t)TempBuffer[0][i] - 48) * h;
			h *= 10;
		}
		len2 = col;
		h = 0.1;
		for(int i = 0; i < len2; i++)
		{
			p2 += ((uint8_t)TempBuffer[1][i] - 48) * h;
			h *= 0.1;
		}
	}
	result = sign*(p1 + p2);
	return result;
}

/** @brief  : Convert value to char array
**  @agr    : Input value, result buffer
**  @retval : 0 or 1
**/
uint8_t ToChar(double value, uint8_t *pBuffer, int NbAfDot)
{
	uint32_t BefD;
	double AftD;
	uint8_t buffer[20], index = 0, strleng = 0, len = 0;
	if(value < 0)
	{
		value = -value;
		pBuffer[0] = (uint8_t)'-';
		index++;
		strleng++;
	}
	BefD = (uint32_t)value;
	AftD = value - BefD;
	if (BefD == 0) 
	{
		pBuffer[index] = (uint8_t)'0';
		index++;
		strleng++;
	}
	else
	{
		while(BefD != 0)
		{
			buffer[len] = (BefD % 10) + 48;
			BefD /= 10;
			len++;
		}
		strleng = index + len;
		// take the befD value
		for (int i = 0; i < len; i++)
		{
			pBuffer[index + i] = buffer[len - i - 1];
		}
	}

	if(value != (int)value)
	{
		pBuffer[strleng] = (uint8_t)'.';
		strleng++;
		for (int i = 0; i < NbAfDot; i++)
		{
			AftD *= 10;
			pBuffer[strleng + i] = (uint8_t)AftD + 48;
			AftD = AftD - (uint8_t)AftD;
		}
		strleng += NbAfDot;
	}
	return strleng;
}

/** @brief  : LRC calculate
**  @agr    : Input string array and its lenght
**  @retval : Result
**/
uint8_t LRCCalculate(uint8_t *pBuffer, int length)
{
	unsigned int result = 0;
	for(int i = 0; i < length; i++)
	{
		result ^= pBuffer[i];
	}
	return result;
}

/** @brief  : Convert decimal value to hexadecimal
**  @agr    : Input value
**  @retval : Result
**/
uint8_t ToHex(uint8_t input)
{
	input += (input < 10) ? 48 : 55;
	return input;
}

/** @brief  : Check sum and return OK or Not OK
**  @agr    : Input message, 4 higher bits and 4 lower bits of byte
**  @retval : Check status
**/
enum_Status IsCorrectMessage(uint8_t *inputmessage, int length, uint8_t byte1, uint8_t byte2)
{
	uint8_t CheckSum, c1, c2;
	CheckSum = LRCCalculate(inputmessage, length);
	c1 = (CheckSum & 0xF0) >> 4;
	c2 = CheckSum & 0x0F;
	c1 = ToHex(c1);
	c2 = ToHex(c2);
	return ((c1 == byte1) && (c2 == byte2)) ? Check_OK : Check_NOK;
}

/** @brief  : Compare 2 input string
**  @agr    : Input string
**  @retval : 1 - equal, 0 - not equal
**/
enum_Status StringHeaderCompare(char *s, char *ref)
{
	int i = 0;
	while(s[i] != 0)
	{
		if(s[i] != ref[i]) 
			return Check_NOK;
		++i;
	}
	return Check_OK;
}

/** @brief  : Get command number of receive message
**  @agr    : Input message
**  @retval : None 
**/
enum_Command	Veh_MsgToCmd(char *U6_message)
{
	if(StringHeaderCompare(U6_message, "$VEHCF"))      
		return Vehicle_Config;
	else if(StringHeaderCompare(U6_message, "$TSAMP"))
		return Sample_Time; 
	else if(StringHeaderCompare(U6_message, "$TSEND"))
		return SendData_Time; 
	else if(StringHeaderCompare(U6_message, "$IMUCF"))
		return IMU_Config; 
	else if(StringHeaderCompare(U6_message, "$SFRST")) //F10
		return Soft_Reset; 
	else if(StringHeaderCompare(U6_message, "$MACON"))
		return Manual_Config; 
	else if(StringHeaderCompare(U6_message, "$AUCON"))
		return Auto_Config; 
	else if(StringHeaderCompare(U6_message, "$VPLAN"))
		return Path_Plan; 	 
	else if(StringHeaderCompare(U6_message, "$FSAVE"))
		return Flash_Save; 
	else if(StringHeaderCompare(U6_message, "$KCTRL"))
		return KeyBoard_Control;
	else 
		return None;
}

/** @brief  : Feed back message
**  @agr    : Result and status
**  @retval : lenght 
**/
int FeedBack(uint8_t *outputmessage, char inputstring[MAX_LORA_BUFFERSIZE])
{
	int i = 0;
	while(inputstring[i] != 0)
	{
		outputmessage[i] = inputstring[i];
		++i;
	}
	outputmessage[i++]  = 0x0D;	// carriage return
	outputmessage[i++]  = 0x0A; // line feed
	return i;
}

void Convert_Double_Array(double *pInputArray, int n)
{
	double temp;
	for(int i = 0; i < (n/2); i++)
	{
		temp = pInputArray[i];
		pInputArray[i] = pInputArray[n - 1 - i];
		pInputArray[n - 1 - i] = temp;
	}
}

/*-------------------- Stanley Function ------------------*/
/** @brief  : Convert m/s to roll/minute
**  @agr    : Input velocity in m/s from panel
**  @retval : RPM value
**/
double MPS2RPM(double vel)
{
	return ((vel / (2 * pi * Wheel_Radius)) * 60);
}


/** @brief  : Pi to Pi
**  @agr    : input angle
**  @retval : double value
**/
double Pi_To_Pi(double angle)
{
	double result;
	if(angle > pi)
	{
		result = angle - 2 * pi;
	}
	else if (angle < -pi)
	{
		result = angle + 2 * pi;
	}
	else
	{
		result = angle;
	}
	return result;
}

double Degree_To_Degree(double angle)
{
	if(angle > 180)
		angle = angle - 360;
	else if(angle < -180)
		angle = angle + 360;

	return angle;
}
/** @brief  : Convert lat lon coordinate into UTM
**  @agr    : input lat and lon values from GNGLL message GLONASS Lat Lon
**  @retval : Result buffer x ,y
**/
void GPS_LatLonToUTM(GPS *pgps)
{
	double la, lo, lat, lon, sa, sb, e2, e2cuadrada, c, Huso, S, deltaS, a, epsilon, nu, v, ta, a1, a2, j2, j4, j6, alfa, beta, gama, Bm, xx, yy;
	la = pgps->Latitude;
	lo = pgps->Longitude;
	sa = 6378137.00000;
	sb = 6356752.314245;
	e2 = pow(pow(sa,2) - pow(sb,2),0.5) / sb;
	e2cuadrada = pow(e2,2);
	c = pow(sa,2) / sb;
	lat = la * (pi / 180);
	lon = lo * (pi / 180);
	Huso = (double)((int) ((lo / 6) + 31) );
	S = ((Huso * 6) - 183);
	deltaS = lon - (S * (pi / 180));
	a = cos(lat) * sin(deltaS);
	epsilon = 0.5 * log((1 + a) / (1 - a));
	nu = atan(tan(lat) / cos(deltaS)) - lat;
	v = (c / pow((1 + (e2cuadrada * pow(cos(lat),2))),0.5)) * 0.9996;
	ta = (e2cuadrada / 2) * pow(epsilon,2) * pow(cos(lat),2);
	a1 = sin(2 * lat);
	a2 = a1 * pow(cos(lat),2);
	j2 = lat + (a1 / 2);
	j4 = ((3 * j2) + a2) / 4;
	j6 = ((5 * j4) + (a2 * pow(cos(lat),2))) / 3;
	alfa = ((double)3 / (double)4) * e2cuadrada;
	beta = ((double)5 / (double)3) * pow(alfa,2);
	gama = ((double)35 / (double)27) * pow(alfa,3);
	Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	xx = epsilon * v * (1 + (ta / 3)) + 500000;
	yy = nu * v * (1 + ta) + Bm;
	if (yy < 0)
	{
		yy = 9999999 + yy;
	}
	// If dx, dy > 3, then it is noise
	if(pgps->CorX == 0)
	{
		pgps->CorX = xx;
		pgps->CorY = yy;
		pgps->Pre_CorX = xx;
		pgps->Pre_CorY = yy;
		pgps->dx = 0;
		pgps->dy = 0;
		pgps->NewDataAvailable = 1;
		pgps->Times = 0;
	}
	else if((fabs(xx - pgps->CorX) < 3.0) || (fabs(yy - pgps->CorY) < 3.0))
	{
		pgps->Pre_CorX = pgps->CorX;
		pgps->Pre_CorY = pgps->CorY;
		pgps->CorX = xx;
		pgps->CorY = yy;
		pgps->dx = pgps->CorX - pgps->Pre_CorX;
		pgps->dy = pgps->CorY - pgps->Pre_CorY;
		pgps->NewDataAvailable = 1;
		pgps->Times = 0;
	}
}

/** @brief  : Initial value for GPS functions
**  @agr    : input
**  @retval : Return fix value
**/
void GPS_ParametersInit(GPS *pgps)
{
	pgps->CorX = 0;
	pgps->CorY = 0;
	pgps->dx = 0;
	pgps->dy = 0;
	pgps->NewDataAvailable = 0;
	pgps->Times = 0;
	pgps->Latitude = 0;
	pgps->Longitude = 0;
	pgps->Robot_Velocity = 0;
	pgps->NbOfWayPoints = 0;
	pgps->Pre_CorX = 0;
	pgps->Pre_CorY = 0;
	pgps->Goal_Flag = Check_NOK;
	pgps->GPS_Error = Veh_NoneError;
	pgps->K = 0.5;
	pgps->NbOfP = 0;
	pgps->Step = 0.5;
	pgps->dmin = 0;
	pgps->Cor_Index = 0;
	pgps->efa = 0;
}

/** @brief  : GPS updates path yaw 
**  @agr    : GPS
**  @retval : none
**/
void GPS_UpdatePathYaw(GPS *pgps)
{
	for(int i = 0; i < pgps->NbOfP - 1; i++)
	{
		pgps->P_Yaw[i] = atan2(pgps->P_Y[i + 1] - pgps->P_Y[i], pgps->P_X[i + 1] - pgps->P_X[i]);
	}
	pgps->P_Yaw[pgps->NbOfP - 1] = pgps->P_Yaw[pgps->NbOfP - 2];
}

/** @brief  : GPS over write GPS coordinate
**  @agr    : GPS and K
**  @retval : none
**/
void GPS_UpdateCoordinateXY(GPS *pgps, double Cor_X, double Cor_Y)
{
	pgps->CorX = Cor_X;
	pgps->CorY = Cor_Y;
}

/** @brief  : Save GPS path coordinate to internal flash memory
**  @agr    : GPS and FlashMemory
**  @retval : none
**/
void GPS_SavePathCoordinateToFlash(GPS *pgps, FlashMemory *pflash)
{
	pflash->Length = 0;
	pflash->Length += ToChar(pgps->NbOfWayPoints,&pflash->WriteInBuffer[pflash->Length],1);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	for(int i = 0; i < pgps->NbOfWayPoints; i++)
	{
		pflash->Length += ToChar(pgps->Path_X[i],&pflash->WriteInBuffer[pflash->Length],6);
		pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
		pflash->Length += ToChar(pgps->Path_Y[i],&pflash->WriteInBuffer[pflash->Length],6);
		pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	}
	EraseMemory(FLASH_Sector_6);
	WriteToFlash(pflash,FLASH_Sector_6,FLASH_GPSPara_BaseAddr);
}

/** @brief  : Convert lattitude/longtitude in DMS format into DD format
              dd.ff = dd + mm/60 + ss/3600
**  @agr    : Lattitude/Longitude in NMEA format (DMS)
**  @retval : Lattitude/Longitude in DD format (*-1 for W and S) 
**/
double GPS_LLToDegree(double LL)
{
    double dd, mm;
	dd = (int)(LL / 100); 
	mm = LL - (double)(dd * 100);
	return (dd + mm/60);
}

/** @brief  : Function get lat lon value from GNGLL message
**  @agr    : 
        @pgps: pointer to instance of GPS
        @inputmessage: a string represent latitude in ddmm.mmmmm format 
**  @retval : Value
**/
void GPS_StringToLat(GPS *pgps, char *inputmessage)
{
	double s1 = 0, s2 = 0;
	int temp = 1000;
	for(int i = 0; i < 4; i++)
	{
		s1 += (inputmessage[i] - 48) * temp;
		temp /= 10;
	}
	temp = 10000;
	for(int i = 5; i < 10; i++)
	{
		s2 += (inputmessage[i] - 48) * temp;
		temp /= 10;
	}
	s2 /= 100000;
	pgps->Latitude = GPS_LLToDegree(s1 + s2);
}

/** @brief  : Function get lat lon value from GNGLL message
**  @agr    : String value received from message
        @pgps: pointer to instance of GPS
        @inputmessage: a string represent latitude in dddmm.mmmmm format 
**  @retval : Value
**/
void GPS_StringToLng(GPS *pgps, char *inputmessage)
{
	double s1 = 0, s2 = 0;
	int temp = 10000;
	for(int i = 0; i < 5; i++)
	{
		s1 += (inputmessage[i] - 48) * temp;
		s2 += (inputmessage[i + 6] - 48) * temp;
		temp /= 10;
	}
	s2 /= 100000;
	pgps->Longitude = GPS_LLToDegree(s1 + s2);
}

void GPS_ClearPathCorBuffer(GPS *pgps)
{
	for(int i = 0; i < 20; ++i)
	{
		pgps->Path_X[i] = pgps->Path_Y[i] = 0;
	}
}

void GPS_ClearPathBuffer(GPS *pgps)
{
	for(int i = 0; i < MAX_NUM_COORDINATE; ++i)
	{
		pgps->P_X[i] = pgps->P_Y[i] = pgps->P_Yaw[i] = 0;
	}
}

void GPS_PathPlanning(GPS *pgps, float Step)
{
	double temp, a, b;
	pgps->NbOfP = 0;
	for(int i = 0; i < pgps->NbOfWayPoints - 1; i++)
	{
		a = (pgps->Path_Y[i + 1] - pgps->Path_Y[i]) / (pgps->Path_X[i + 1] - pgps->Path_X[i]);
		b = pgps->Path_Y[i] - a * pgps->Path_X[i];
		temp = pgps->Path_X[i];
		if(pgps->Path_X[i] < pgps->Path_X[i + 1])
		{
			while(temp < pgps->Path_X[i + 1])
			{
				pgps->P_X[pgps->NbOfP] = temp;
				pgps->P_Y[pgps->NbOfP] = a * pgps->P_X[pgps->NbOfP] + b;
				pgps->NbOfP++;
				temp += Step;
			}
		}
		else
		{
			while(temp > pgps->Path_X[i + 1])
			{
				pgps->P_X[pgps->NbOfP] = temp;
				pgps->P_Y[pgps->NbOfP] = a * pgps->P_X[pgps->NbOfP] + b;
				pgps->NbOfP++;
				temp -= Step;
			}
		}
	}
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
void GPS_StanleyControl(GPS *pgps, double SampleTime, double M1Velocity, double M2Velocity)
{                   /*   Current pose of the robot   / /  Path coordinate  / /  ThetaP  */
	double dmin = 0,dx,dy,d,posX, posY;
	int 	 index = 0;
	double efa, goal_radius, VM1, VM2, AngleRadian;
	float L = 0.19, Lf=0, Lfc=0.1;
	pgps->Angle = &Mag;
	AngleRadian = pgps->Angle->Angle * (double)pi/180;
	AngleRadian = pi/2 - AngleRadian;
	VM1 = Wheel_Radius * 2 * pi * M1Velocity/60;	
	VM2 = Wheel_Radius * 2 * pi * M2Velocity/60;
	pgps->Robot_Velocity = (VM1 + VM2)/2;
	
	// Calculate new Pos if there is no new data from GPS
	posX = pgps->CorX;
	posY = pgps->CorY;
	if(!pgps->NewDataAvailable)
	{
		pgps->Times++;
		posX = pgps->CorX + pgps->dx * pgps->Times * Timer.T;
		posY = pgps->CorY + pgps->dy * pgps->Times * Timer.T;
	}
	// Calculate the fron wheel position
	posX += L * cos(AngleRadian);
	posY += L * sin(AngleRadian);

	//Searching the nearest point
	//---------------------------
	for(int i = 0; i < pgps->NbOfP; i++)
	{
		dx = posX - pgps->P_X[i];
		dy = posY - pgps->P_Y[i];
		d  = sqrt(pow(dx,2) + pow(dy,2));
		if(i == 0)
		{
			dmin 	= d;
			index = i;
		}
		else
		{
			if(dmin > d) // d is the new value that near the point
			{
				dmin = d;  // min 
				index = i;	// position of the minimum value
			}
		}
	}
	if(index > pgps->NbOfP - 1)
		index -=1;
	Lf = pgps->K * pgps->Robot_Velocity + Lfc;
		while((Lf > L) && (index + 1 < pgps->NbOfP))
	{
		dx = posX - pgps->P_X[index];
		dy = posY - pgps->P_Y[index];
		L = sqrt(pow(dx,2) + pow(dy,2));
		index++;
	}
	pgps->dmin = dmin;
	pgps->P_Yaw_Index = index;
	efa = - ((posX - pgps->P_X[index]) * (cos(AngleRadian + pi/2)) + (posY - pgps->P_Y[index]) * sin(AngleRadian + pi/2));
	pgps->efa = efa;
	goal_radius = sqrt(pow(posX - pgps->P_X[pgps->NbOfWayPoints - 1],2) + pow(posY - pgps->P_Y[pgps->NbOfWayPoints - 1],2));
	if(goal_radius <= 1)
		GPS_NEO.Goal_Flag = Check_OK;
	pgps->Thetae = Pi_To_Pi(AngleRadian - pgps->P_Yaw[index]);
	pgps->Thetad = -atan2(pgps->K* efa, pgps->Robot_Velocity);
	pgps->Delta_Angle  = (pgps->Thetae + pgps->Thetad)*(double)180/pi;
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
void GPS_PursuitControl(GPS *pgps, double SampleTime, double M1Velocity, double M2Velocity)
{                   /*   Current pose of the robot   / /  Path coordinate  / /  ThetaP  */
	double dmin = 0,dx,dy,d,posX, posY;
	int 	 index = 0;
	double Lf, Alpha, goal_radius, VM1, VM2, AngleRadian;
	float L = 0.2, Lfc = 0.6;
	pgps->Angle = &Mag;
	AngleRadian = pgps->Angle->Angle * (double)pi/180;
	AngleRadian = pi/2 - AngleRadian;
	VM1 = Wheel_Radius * 2 * pi * M1Velocity/60;	
	VM2 = Wheel_Radius * 2 * pi * M2Velocity/60;
	pgps->Robot_Velocity = (VM1 + VM2)/2;
	
	// Calculate new Pos if there is no new data from GPS
	posX = pgps->CorX;
	posY = pgps->CorY;
	if(!pgps->NewDataAvailable)
	{
		pgps->Times++;
		posX = pgps->CorX + pgps->dx * pgps->Times * Timer.T;
		posY = pgps->CorY + pgps->dy * pgps->Times * Timer.T;
	}
	// Calculate the fron wheel position
	posX -= L * cos(AngleRadian);
	posY -= L * sin(AngleRadian);
	Lf = pgps->K * pgps->Robot_Velocity + Lfc;
	//Searching the nearest lookahead point
	//---------------------------
	for(int i = 0; i < pgps->NbOfP; i++)
	{
		dx = posX - pgps->P_X[i];
		dy = posY - pgps->P_Y[i];
		d  = sqrt(pow(dx,2) + pow(dy,2));
		if(i == 0)
		{
			dmin 	= d;
			index = i;
		}
		else
		{
			if(dmin > d) // d is the new value that near the point
			{
				dmin 	= d;  // min 
				index = i;	// position of the minimum value
			}
		}
	}
	while((Lf > L) && (index + 1 < pgps->NbOfP))
	{
		dx = posX - pgps->P_X[index];
		dy = posY - pgps->P_Y[index];
		L = sqrt(pow(dx,2) + pow(dy,2));
		index++;
	}
	if(index+1 > pgps->NbOfP)
		index = index - 1;
	pgps->dmin = dmin;
	pgps->P_Yaw_Index = index;
	goal_radius = sqrt(pow(posX - pgps->P_X[pgps->NbOfWayPoints - 1],2) + pow(posY - pgps->P_Y[pgps->NbOfWayPoints - 1],2));
	Alpha = atan2(posY - pgps->P_Y[index], posX - pgps->P_X[index]) - AngleRadian;
	if(goal_radius <= 1)
		GPS_NEO.Goal_Flag = Check_OK;
	pgps->Delta_Angle  = atan2(2*L*sin(Alpha), Lf)*180/pi;
}

/** @brief  : Header compare GPS message
**  @agr    : Input header
**  @retval : Check status
**/
enum_Status	GPS_HeaderCompare(uint8_t *s1, char Header[5])
{
	for(int i = 0; i < 5; i++)
	{
		if(s1[i] != (uint8_t)Header[i])
			return Check_NOK;
	}
	return Check_OK;
}

/** @brief  : Get message from NMEA protocol
**  @agr    : GPS and Inputmessage
**  @retval : None
**/
enum_Error	GPS_GetLLQMessage(GPS *pgps, uint8_t *inputmessage,	char result[MESSAGE_ROW][MESSAGE_COL])
{
	int Message_Index = 0, GxGLL_Index = 0, GxGGA_Index = 0, Length = 0;
	while(inputmessage[Message_Index] != 0 && (Message_Index < ROVER_RX_BUFFERSIZE))
	{
		/* Because GxGGA come before GxGLL */
		if(inputmessage[Message_Index] == (uint8_t)'$')
		{
			if( (GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GNGGA")) || 
				(GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GPGGA")) )
			{
				GxGGA_Index = Message_Index;
			}
			else if( (GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GNGLL")) ||
					 (GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GPGLL")) )
			{
				GxGLL_Index = Message_Index;
				break;
			}
		}
		++Message_Index;
	}

	if(GxGLL_Index != 0)
	{
		Length = LengthOfLine(&inputmessage[GxGLL_Index]);
		GetMessageInfo( (char *)&inputmessage[GxGLL_Index], result, ',');
		if( IsCorrectMessage(&inputmessage[GxGLL_Index + 1], Length - 4, 
			inputmessage[GxGLL_Index + Length - 2], 
			inputmessage[GxGLL_Index + Length - 1]) )
		{
			if(result[6][0] == 'A') // Block STATUS, 'A': data valid, 'V': data not valid
			{
				// Latitude range is from 0 to 90 and longitude is range from 0 to 180.
				GPS_StringToLat(&GPS_NEO, &result[1][0]); 
				GPS_StringToLng(&GPS_NEO, &result[3][0]); 
				GPS_LatLonToUTM(&GPS_NEO);
			} else 
				return Veh_InvalidGxGLLMessage_Err;
		} else 
			return Veh_GxGLLCheckSum_Err;
	} else
		return Veh_ReadGxGLLMessage_Err;

	if(GxGGA_Index != 0)
	{
		Length = LengthOfLine(&inputmessage[GxGGA_Index]);
		GetMessageInfo( (char *)&inputmessage[GxGGA_Index], result, ',');
		if( IsCorrectMessage(&inputmessage[GxGGA_Index + 1], Length - 4, 
			inputmessage[GxGGA_Index + Length - 2], 
			inputmessage[GxGGA_Index + Length - 1]) )
		{
			pgps->GPS_Quality = (enum_GPS_Quality) GetValueFromString(&result[6][0]);
		} else 
			return Veh_GxGGACheckSum_Err;
	} else 
		return Veh_ReadGxGGAMessage_Err;

	return Veh_NoneError;
}

/** @brief  : Get message from GNGLL/GPGLL 
**  @agr    : GPS and Inputmessage
**  @retval : None
**/
enum_Status	GPS_GetCoordinateMessage(uint8_t *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL])
{
	int index = 0;
	enum_Status flag = Check_NOK;
	while(inputmessage[index] != 0)
	{
		if(inputmessage[index] == '$')
		{
			if( ((inputmessage[index + 1] == 'G') && 
				 (inputmessage[index + 2] == 'N') && 
				 (inputmessage[index + 3] == 'G') && 
				 (inputmessage[index + 4] == 'L') && 
				 (inputmessage[index + 5] == 'L')) || 
				((inputmessage[index + 1] == 'G') && 
				 (inputmessage[index + 2] == 'P') && 
				 (inputmessage[index + 3] == 'G') && 
				 (inputmessage[index + 4] == 'L') && 
				 (inputmessage[index + 5] == 'L')) )
			{
				flag = Check_OK;
				break;
			}
			else index++;
		}
		else index++;
	}
	if(flag)
		GetMessageInfo( (char *)&inputmessage[index], result,',');
	else return Check_NOK;
	return IsCorrectMessage(&inputmessage[index + 1], 46, (uint8_t)result[7][2], (uint8_t)result[7][3]);
}

/** @brief  : Get quality value from GNGGA/GPGGA message
**  @agr    : GPS and Inputmessage
**  @retval : None
**/
enum_Status GPS_GetQualityFromString(GPS *pgps, uint8_t *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL]) 
{
	int index = 0;
	enum_Status flag = Check_NOK;
	while(inputmessage[index] != 0)
	{
		if(inputmessage[index] == '$')
		{
			if( ((inputmessage[index + 1] == 'G') && 
				 (inputmessage[index + 2] == 'N') && 
				 (inputmessage[index + 3] == 'G') && 
				 (inputmessage[index + 4] == 'G') && 
				 (inputmessage[index + 5] == 'A')) || 
				((inputmessage[index + 1] == 'G') && 
				 (inputmessage[index + 2] == 'P') && 
				 (inputmessage[index + 3] == 'G') && 
				 (inputmessage[index + 4] == 'G') && 
				 (inputmessage[index + 5] == 'A')) )
			{
				flag = Check_OK;
				break;
			} else 
				index++;
		} else 
			index++;
	}
	if(flag)
		GetMessageInfo((char*)&inputmessage[index],result,',');
	else 
		return Check_NOK;
	return IsCorrectMessage(&inputmessage[index + 1],LengthOfLine(&inputmessage[index + 1]) - 3, (uint8_t)result[14][5], (uint8_t)result[14][6]);
}

/*-------------------- Fuzzy control -----------------------*/
/* Variables of fuzzy control block */

/** @brief  : Find maximum number
**  @agr    : 2 input values
**  @retval : Output maximum value
**/
double Fuzzy_Min(double in1, double in2)
{
	double min;
	min = in1;
	if(min > in2) min = in2;
	return min;
}

double Fuzzy_Max(double *input,int len)
{
	double max;
  max = input[0];
	for(int i = 1; i < len; i++)
	{
		if(max < input[i])
		{
			max = input[i];
		}
	}
	return max;
}

/** @brief  : Trapf function
**  @agr    : 4 parameters (left to right)
**  @retval : Value relates to x
**/
double Trapf(trapf *ptrapf, double x)
{
	double result;
	if(x < ptrapf->h1) result = 0;
	else if(x < ptrapf->h2) result = (x - ptrapf->h1) / (ptrapf->h2 - ptrapf->h1);
	else if(x < ptrapf->h3) result = 1;
	else if(x < ptrapf->h4) result = (ptrapf->h4 - x) / (ptrapf->h4 - ptrapf->h3);
	else result = 0;
	return result;
}

/** @brief  : Trimf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
double Trimf(trimf *ptrimf, double x)
{
	double result;
	if(x < ptrimf->a1) result = 0;
	else if(x < ptrimf->a2) result = (x - ptrimf->a2) / (ptrimf->a2 - ptrimf->a1);
	else if(x < ptrimf->a3) result = (ptrimf->a3 - x) / (ptrimf->a3 - ptrimf->a2);
	else result = 0;
	return result;
}

/** @brief  : Update Paramter for Trimf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Trimf_Update(trimf *ptrimf, double a1, double a2, double a3)
{
	ptrimf->a1 = a1;
	ptrimf->a2 = a2;
	ptrimf->a3 = a3;
}

/** @brief  : Update Paramerters for trapf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Trapf_Update(trapf *ptrapf, double a1, double a2, double a3, double a4)
{
	ptrapf->h1 = a1;
	ptrapf->h2 = a2;
	ptrapf->h3 = a3;
	ptrapf->h4 = a4;
}

/** @brief  : Fuzzy init parameter procedure
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Fuzzy_ParametersInit(void)
{
	/*   Input 1 (e = Set_theta - theta)  */
		// NB : -2 - -0.17
		Trapf_Update(&In1_NB,-2,-1,-0.22,-0.17);
		// NS : 0.15 - 0.45
		Trimf_Update(&In1_NS,-0.22,-0.11,-0.044);
		// ZE : 0 - 0.2
		Trimf_Update(&In1_ZE,-0.056,0,0.056);
		// PS : 0.15 - 0.45
		Trimf_Update(&In1_PS,0.044,0.11,0.22);
		// PB : 0.4 - 1
		Trapf_Update(&In1_PB,0.17,0.22,1,2);
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE : 0.3 - 1
		Trapf_Update(&In2_NE,-2,-1,-0.4,-0.3);
		// ZE : 0 - 0.4
		Trimf_Update(&In2_ZE,-0.4,0,0.4);
		// PO : 0.3 - 1
		Trapf_Update(&In2_PO,0.3,0.4,1,2);
		/* Output value */
		NB = -0.95;
		NM = -0.8;
		NS = -0.4;
		ZE = 0;
		PS = 0.4;
		PM = 0.8;
		PB = 0.95;
}

void	SelectFuzzyOutput(double vel)
{
	/*----------Fuzzy parameter init ------------------*/
	if (vel < MPS2RPM(0.3))
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB : 0.4 - 1
		In1_NB.h1 = -2;
		In1_NB.h2 = -1;
		In1_NB.h3 = -0.22;
		In1_NB.h4 = -0.17;
		// NS : 0.15 - 0.45
		In1_NS.a1 = -0.22;
		In1_NS.a2 = -0.11;
		In1_NS.a3 = -0.044;
		// ZE : 0 - 0.2
		In1_ZE.a1 = -0.056;
		In1_ZE.a2 = 0;
		In1_ZE.a3 = 0.056;
		// PS : 0.15 - 0.45
		In1_PS.a1 = 0.044;
		In1_PS.a2 = 0.11;
		In1_PS.a3 = 0.22;
		// PB : 0.4 - 1
		In1_PB.h1 = 0.17;
		In1_PB.h2 = 0.22;
		In1_PB.h3 = 1;
		In1_PB.h4 = 2;
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE : 0.3 - 1
		In2_NE.h1 = -2;
		In2_NE.h2 = -1;
		In2_NE.h3 = -0.4;
		In2_NE.h4 = -0.3;
		// ZE : 0 - 0.4
		In2_ZE.a1 = -0.4;
		In2_ZE.a2 = 0;
		In2_ZE.a3 = 0.4;
		// PO : 0.3 - 1
		In2_PO.h1 = 0.3;
		In2_PO.h2 = 0.4;
		In2_PO.h3 = 1;
		In2_PO.h4 = 2;
		/* Output value */
		NB = -0.9;
		NM = -0.75;
		NS = -0.5;
		ZE = 0;
		PS = 0.5;
		PM = 0.75;
		PB = 0.9;
	}
	else
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB : 0.4 - 1
		In1_NB.h1 = -2;
		In1_NB.h2 = -1;
		In1_NB.h3 = -0.45;
		In1_NB.h4 = -0.4;
		// NS : 0.15 - 0.45
		In1_NS.a1 = -0.45;
		In1_NS.a2 = -0.2;
		In1_NS.a3 = -0.15;
		// ZE : 0 - 0.2
		In1_ZE.a1 = -0.2;
		In2_ZE.a2 = 0;
		In2_ZE.a3 = 0.2;
		// PS : 0.15 - 0.45
		In1_PS.a1 = 0.15;
		In1_PS.a2 = 0.2;
		In1_PS.a3 = 0.45;
		// PB : 0.4 - 1
		In1_PB.h1 = 0.4;
		In1_PB.h2 = 0.45;
		In1_PB.h3 = 1;
		In1_PB.h4 = 2;
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE : 0.3 - 1
		In2_NE.h1 = -2;
		In2_NE.h2 = -1;
		In2_NE.h3 = -0.4;
		In2_NE.h4 = -0.3;
		// ZE : 0 - 0.4
		In2_ZE.a1 = -0.4;
		In2_ZE.a2 = 0;
		In2_ZE.a3 = 0.4;
		// PO : 0.3 - 1
		In2_PO.h1 = 0.3;
		In2_PO.h2 = 0.4;
		In2_PO.h3 = 1;
		In2_PO.h4 = 2;
		/* Output value */
		NB = -0.75;
		NM = -0.4;
		NS = -0.175;
		ZE = 0;
		PS = 0.175;
		PM = 0.4;
		PB = 0.75;
	}
}

/** @brief  : Defuzzification Max Min sugeno
**  @agr    : 2 input value
**  @retval : Output value
**/

void	Defuzzification_Max_Min(IMU *pimu)
{
	double pBeta[5], num = 0, den = 0, temp;
	//NB and NE is NB
	pBeta[0] = Fuzzy_Min(Trapf(&In1_NB,pimu->Fuzzy_Error), Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	num = NB * pBeta[0];
	den = pBeta[0];
	//NS and NE is NM
	//NB and ZE is NM
	pBeta[0] = Fuzzy_Min(Trimf(&In1_NS,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trapf(&In1_NB,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,2);
	num += NM * temp;
	den += temp;
	//ZE and NE is NS
	//NS and ZE is NS
	//NB and PO is NS
	pBeta[0] = Fuzzy_Min(Trimf(&In1_ZE,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_NS,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[2] = Fuzzy_Min(Trapf(&In1_NB,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,3);
	num += NS * temp;
	den += temp;
	//PS and NE is ZE
	//ZE and ZE is ZE
	//NS and PO is ZE
	pBeta[0] = Fuzzy_Min(Trimf(&In1_PS,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_ZE,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[2] = Fuzzy_Min(Trimf(&In1_NS,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,3);
	num += ZE * temp;
	den += temp;
	//PB and NE is PS
	//PS and ZE is PS
	//ZE and PO is PS
	pBeta[0] = Fuzzy_Min(Trapf(&In1_PB,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_PS,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[2] = Fuzzy_Min(Trimf(&In1_ZE,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,3);
	num += PS * temp;
	den += temp;
	//PB and ZE is PM
	//PS and PO is PM
	pBeta[0] = Fuzzy_Min(Trapf(&In1_PB,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_PS,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,2);
	num += PM * temp;
	den += temp;
	//PB and PO is PB
	pBeta[0] = Fuzzy_Min(Trapf(&In1_PB,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	num += PB * pBeta[0];
	den += pBeta[0];
	if(den == 0) pimu->Fuzzy_Out = 0;
	else
	{
		pimu->Fuzzy_Out = num / den;
	}
}


/*------------------------ Flash read/write function ------------------*/

/*------------------------ IMU functions ------------------*/
/** @brief  : Get data from IMU message
**  @agr    : inputmessage, val
**  @retval : Output value
**/
void	IMU_ParametesInit(IMU *pimu)
{
	pimu->Angle 					= 0;
	pimu->Set_Angle 				= 0;
	pimu->Pre_Angle 				= 0;

	pimu->Fuzzy_Out 				= 0;
	pimu->Fuzzy_Error 				= 0;
	pimu->Fuzzy_Error_dot			= 0;
}

/** @brief  : Update Set angle
**  @agr    : IMU and Angle
**  @retval : none
**/
void	IMU_UpdateSetAngle(IMU *pimu, double ComAngle)
{
	double temp;
	temp = pimu->Angle + ComAngle;
	pimu->Set_Angle = Degree_To_Degree(temp);
}

/** @brief  : Update previous angle
**  @agr    : IMU
**  @retval : none
**/
void	IMU_UpdatePreAngle(IMU *pimu)
{
	pimu->Pre_Angle = pimu->Angle;
}


/** @brief  : Update input for fuzzy controller
**  @agr    : imu and sampletime
**  @retval : none
**/
void	IMU_UpdateFuzzyInput(IMU *pimu, double *pSampleTime)
{
	pimu->Fuzzy_Error 		= pimu->Set_Angle - pimu->Angle;
	pimu->Fuzzy_Error_dot = -(pimu->Angle - pimu->Pre_Angle)/(*pSampleTime);
	if(pimu->Fuzzy_Error > 180) 
		pimu->Fuzzy_Error -= 360;
	else if(pimu->Fuzzy_Error < -180) 
		pimu->Fuzzy_Error += 360;
	pimu->Fuzzy_Error 		*= pimu->Ke;
	pimu->Fuzzy_Error_dot *= pimu->Kedot;
}

/** @brief  : Update fuzzy coefficients
**  @agr    : imu and Ke,kedot,ku
**  @retval : none
**/
void	IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku)
{
	pimu->Ke		= Ke;
	pimu->Kedot 	= Kedot;
	pimu->Ku 		= Ku;
}

/** @brief  : Get data from IMU message, update IMU angle
**  @agr    : 
**      @pimu: instance of struct IMU
**      @inputmessage: USART 2 RX BUFFER, contain information receiving from IMU
**          format "0x0A |roll |pitch |yaw |0x0D"
**          unit of angle: mdeg
**  @retval : Output value
**/
enum_Error IMU_GetValueFromMessage(IMU *pimu, uint8_t *inputmessage)
{
	int temp = 100000;
	double Angle = 0;
	if(inputmessage[0] == 0x0A)
	{
		for(int i = 0; i < 6; i++)
		{
			Angle += (inputmessage[IMU_AngleIndex + 1 + i] - 48) * temp;
			temp /= 10;
		}
		Angle /= 1000;
		
		pimu->Angle = (inputmessage[IMU_AngleIndex] == (uint8_t)' ') ? Angle : -Angle;
		return Veh_NoneError;
	}
	else
	{
		pimu->Angle = pimu->Pre_Angle;
		return IMU_WrongMessage;
	}
}

/*---------------- Read / Write Flash Memory Functions -----------------*/
/** @brief  : Convert 4 bytes (in char) to a Word data in order to save in flash memory
**  @agr    : Input array of bytes data
**  @retval : Number of 32 bits address
**/
int Convert4BytesToWordData(uint8_t *pInput, uint32_t *pOutBuffer, int InputLength)
{
	int length = 0, h = 0;
	if((InputLength % 4) != 0)
		length = InputLength / 4 + 1;
	else
		length /= 4;
	for(int i = 0; i < length; i++)
	{
		for(int j = 3; j >= 0; j--)
		{
			pOutBuffer[i] += pInput[h] << (8 * j);
			h++;
		}
	}
	return length;
}

/** @brief  : Write word data to Flash memory
**  @agr    : Flash_Sector, Sector_BaseAddr and input word data pointer
**  @retval : None
**/
void WriteToFlash(FlashMemory *pflash, uint32_t FLASH_Sector, uint32_t FLASH_BaseAddr)
{
	for(int i = 0; i < 100; i++)
	{
		pflash->WriteIn32bBuffer[i] = 0;
	}
	pflash->WriteIn32bBuffer[0] = Convert4BytesToWordData(pflash->WriteInBuffer,&pflash->WriteIn32bBuffer[1],pflash->Length);
	FLASH_Unlock();
	FLASH_ProgramWord(FLASH_BaseAddr,pflash->WriteIn32bBuffer[0]);
	FLASH_BaseAddr += 4;
	for(int i = 0; i < pflash->WriteIn32bBuffer[0]; i++)
	{
		FLASH_ProgramWord(FLASH_BaseAddr,pflash->WriteIn32bBuffer[i + 1]);
		FLASH_BaseAddr += 4;
	}
	FLASH_Lock();
}

/** @brief  : Read from flash memory
**  @agr    : input and output message
**  @retval : None
**/
void ReadFromFlash(FlashMemory *pflash, uint32_t FLASH_BaseAddr)
{
	int i = 0;
	uint32_t mask;
	int length = (int)(*(uint32_t*)FLASH_BaseAddr); /* 4 first byte hold the length */
	for(int count = 0; count < length; count++)
	{
		mask = 0xFF000000;
		FLASH_BaseAddr += 4;
		for(int j = 3; j >= 0; j--)
		{
			pflash->ReadOutBuffer[i] = (uint8_t)(((*(uint32_t*)FLASH_BaseAddr) & mask) >> (8 * j));
			mask >>= 8;
			i++;
		}
	}
}

/** @brief  : Erase sector flash memory
**  @agr    : None
**  @retval : None
**/
void EraseMemory(uint32_t Flash_Sector)
{
	FLASH_Unlock();
	FLASH_EraseSector(Flash_Sector, FLASH_ProgramType_Word);
	FLASH_Lock();
}

