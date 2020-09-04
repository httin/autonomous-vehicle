#include "functions.h"

/* Global variables */

/*----- Stanley Variables -----*/
/*----- Robot Parameter -------*/
Error	        Veh_Error;
Time            Timer;
Status          VehStt;
FlashMemory     Flash;
DCMotor         M1, M2;
IMU	            Mag;
GPS             GPS_NEO;
Message         U1, U2, U6;
Vehicle	        Veh;
double          NB, NM, NS, ZE, PS, PM, PB; // Sugeno output 
trimf           In1_NS, In1_ZE, In1_PS, In2_ZE;
trapf           In1_NB, In1_PB, In2_NE, In2_PO;
/*----- Error functions -----------------------------------*/
void Error_AppendError(Error *perror, enum_Error err)
{
	perror->Error_Buffer[perror->Error_Index % 40] = err;
	++perror->Error_Index;
}
/*----- Init, and function to config vehicle status -------*/
void Status_ParametersInit(Status *pStatus)
{
	pStatus->Veh_Enable_SendData = Check_NOK;
}

/** @brief  : Function compute PID value 
**	@agr    : void
**	@retval : None
**/
double filter(double alpha, double x, double pre_x)
{
	return (1 - alpha)*x + alpha*pre_x;
}

void PID_Compute(DCMotor *ipid, Time* pTime)
{
	ipid->Error = ipid->current_set_v - ipid->current_v; // sample e(k)

	ipid->PID_Out = ipid->Pre_PID + 
		ipid->Kp * (ipid->Error - ipid->Pre_Error) + 
		0.5 * ipid->Ki * (pTime->velocity_T) * (ipid->Error + ipid->Pre_Error) + 
		(ipid->Kd / (pTime->velocity_T)) * (ipid->Error - 2 * ipid->Pre_Error + ipid->Pre2_Error);

	/* High Pass Filter */
	ipid->PID_Out = filter(0.08, ipid->PID_Out, ipid->Pre_PID);

	if (ipid->PID_Out < -MAX_PWM)
		ipid->PID_Out = -MAX_PWM;
	else if (ipid->PID_Out > MAX_PWM)
		ipid->PID_Out = MAX_PWM;

	ipid->Pre2_Error = ipid->Pre_Error; // e(k-2) = e(k-1)
	ipid->Pre_Error = ipid->Error; // e(k-1) = e(k)
	ipid->Pre_PID = ipid->PID_Out; // u(k-1) = u(k)
}

void PID_Continious(DCMotor *ipid, Time* pTime)
{
	ipid->Error = ipid->current_set_v - ipid->current_v; // sample e(k)

	ipid->PID_Out = ipid->Kp * ipid->Error
					+ ipid->Ki * (ipid->Error + ipid->Pre_Error)
					+ ipid->Kd * (ipid->Error - ipid->Pre_Error);

	ipid->PID_Out = filter(0.08, ipid->PID_Out, ipid->Pre_PID);

	if (ipid->PID_Out < -MAX_PWM)
		ipid->PID_Out = -MAX_PWM;
	else if (ipid->PID_Out > MAX_PWM)
		ipid->PID_Out = MAX_PWM;

	ipid->Pre_Error = ipid->Error; // e(k-1) = e(k)
}


void PID_UpdateSetVel(DCMotor* pMotor, double target_v)
{
	pMotor->target_v = target_v;
	pMotor->delta_v = (pMotor->target_v - pMotor->current_set_v) / 5;
	pMotor->current_set_v = pMotor->target_v;
}

/** @brief  : PID update parameters function
**	@retval : None
**/
void PID_ParametersUpdate(DCMotor *ipid, double Kp, double Ki, double Kd)
{
	ipid->Kp = Kp;
	ipid->Ki = Ki;
	ipid->Kd = Kd;
}

/** @brief  : reset PID
**	@retval : None
**/
void	PID_ResetPID(DCMotor *ipid)
{
	ipid->Pre_PID = 0;    // e(k)
	ipid->Pre_Error = 0;  // e(k-1)
	ipid->Pre2_Error = 0; // e(k-2)
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
**            Ex: "$ab,cd\r\n" -> length = 6, "$ab,cd\0" -> length = 6
**  @agr    : Input buffer
**  @retval : Length
**/
int	LengthOfLine(uint8_t *inputmessage)
{
	int length = 0;
	while(	(inputmessage[length] != 0) &&
			(inputmessage[length] != 0x0D) && (inputmessage[length + 1] != 0x0A))
	{
		++length;
	}
	return length;
}

double sampleTimeCalc(uint32_t TIMx_Freq, uint16_t pres, uint32_t period)
{
	double res;
	res = ((double)(pres * period)) / (TIMx_Freq);
	return res;
}

/*------------------------ Vehicle Status Function ----------------------------*/
enum_Error	Veh_SplitMsg(uint8_t *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL])
{
	int length = LengthOfLine(inputmessage); 
	if(IsCorrectMessage((uint8_t*)&inputmessage[1], length - 3, inputmessage[length - 2], inputmessage[length - 1]))
	{
		GetMessageInfo((char*)inputmessage, result, ',');
		return Veh_NoneError;
	}
	else
		return LORA_WrongCheckSum;
}

/** @brief  : Seperating each info of message by ','
 ** @agr    : input message, result buffer
 ** @retval : None
 **/
void GetMessageInfo(char *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL], char character)
{
	int col = 0, row = 0, index = 0;

	for(int i = 0; (i < MESSAGE_ROW) && (result[i][0] != 0); ++i)
		for(int j = 0; (j < MESSAGE_COL); ++j)
			result[i][j] = 0;

	while(inputmessage[index] != 0)
	{
		if ( (inputmessage[index] != 0x0D) && (inputmessage[index + 1] != 0x0A) )
		{
			if(inputmessage[index] != character)
			{
				result[row][col] = inputmessage[index];
				++col;
			}
			else 
			{
				++row;
				col = 0;
			}
			++index;
		} else
			break;
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
	char TempBuffer[2][30] = {0};
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
	uint8_t reverse_buffer[20], reverse_length = 0, index = 0, strleng = 0;
	if(value < 0)
	{
		value = -value;
		pBuffer[index++] = (uint8_t)'-';
	}
	BefD = (uint32_t)value;
	AftD = value - BefD;

	if (BefD == 0) 
	{
		pBuffer[index++] = (uint8_t)'0';
		strleng = index;
	}
	else
	{
		while(BefD != 0)
		{
			reverse_buffer[reverse_length] = (BefD % 10) + 48;
			BefD /= 10;
			reverse_length++;
		}
		strleng = index + reverse_length;
		// take the befD value
		for (int i = 0; i < reverse_length; i++)
		{
			pBuffer[index + i] = reverse_buffer[reverse_length - i - 1];
		}
	}
    /* value is double */
	if(value != (uint32_t)value)
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
**			  Message Format: $<TOPIC>,<CONTENT>,<CHECKSUM>\r\n
			  Checksum is XOR bytes from start of <TOPIC> to ',' before <CHECKSUM>
			  Example: $VEHCF,DATA,1,53\r\n
			  Checksum is 'V ^ E ^ H ^ C ^ F ^ , ^ D ^ A ^ T ^ A ^ , ^ 1 ^ ,' = 53
**  @retval : 1 byte checksum
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
	else if(StringHeaderCompare(U6_message, "$TSAMP")) /* TODO */
		return Sample_Time; 
	else if(StringHeaderCompare(U6_message, "$TSEND")) /* TODO */
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
	else if(StringHeaderCompare(U6_message, "$FSAVE")) /* TODO */
		return Flash_Save; 
	else if(StringHeaderCompare(U6_message, "$KCTRL"))
		return KeyBoard_Control;
	else 
		return None;
}

/** @brief  : copy from @ref_str to @out_str until meet \r\n
**  @agr    : Result and status
**  @retval : lenght 
**/
int FeedBack(uint8_t *out_str, char *ref_str)
{
	int i = 0;
	while(ref_str[i] != 0) 
	{
		out_str[i] = ref_str[i];

		if(out_str[i] == 0x0A) // end condition
		{
			if(out_str[i - 1] == 0x0D)
				return i + 1;
			break;
		}
		++i;
	}
	return 0;
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
/** @brief  : Convert m/s to revolution per minute
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
	if(angle > pi)
		angle = angle - 2 * pi;
	else if (angle < -pi)
		angle = angle + 2 * pi;

	return angle;
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
	double la, lo, lat, lon, sa, sb, e2, e2cuadrada, c, Huso, S, deltaS, a, epsilon, 
		nu, v, ta, a1, a2, j2, j4, j6, alfa, beta, gama, Bm, xx, yy, dx, dy, coslat;
	la = pgps->Latitude;
	lo = pgps->Longitude;
	sa = 6378137.00000;
	sb = 6356752.314245;
	e2 = pow((sa*sa) - (sb*sb), 0.5) / sb;
	e2cuadrada = e2*e2;
	c = (sa*sa) / sb;
	lat = la * (pi / 180); // convert from degree to radian
	lon = lo * (pi / 180);
	coslat = cos(lat);
	Huso = ((int) ((lo / 6) + 31) );
	S = ((Huso * 6) - 183);
	deltaS = lon - (S * (pi / 180));
	a = coslat * sin(deltaS);
	epsilon = 0.5 * log((1 + a) / (1 - a));
	nu = atan(tan(lat) / cos(deltaS)) - lat;
	v = (c / pow((1 + (e2cuadrada * pow(coslat,2))), 0.5)) * 0.9996;
	ta = (e2cuadrada / 2) * (epsilon*epsilon) * (coslat*coslat);
	a1 = sin(2 * lat);
	a2 = a1 * (coslat*coslat);
	j2 = lat + (a1 / 2);
	j4 = ((3 * j2) + a2) / 4;
	j6 = ((5 * j4) + (a2 * (coslat*coslat))) / 3;
	alfa = ((double)3 / (double)4) * e2cuadrada;
	beta = ((double)5 / (double)3) * (alfa*alfa);
	gama = ((double)35 / (double)27) * (alfa*alfa*alfa);
	Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	xx = epsilon * v * (1 + (ta / 3)) + 500000;
	yy = nu * v * (1 + ta) + Bm;
	if (yy < 0)
	{
		yy = 9999999 + yy;
	}
	
	dx = xx - pgps->currentPosX; // dx(k) = x(k) - x(k-1)
	dy = yy - pgps->currentPosY; // dy(k) = y(k) - y(k-1)
	/* because CorX = CorY = 0 in the first time */
	if(pgps->CorX == 0)
	{
		pgps->currentPosX = pgps->CorX = xx;
		pgps->currentPosY = pgps->CorY = yy;
		pgps->NewDataAvailable = 1;
	}
	else if(sqrt(dx*dx + dy*dy) < 1) 
	{
		pgps->currentPosX = pgps->CorX = xx;
		pgps->currentPosY = pgps->CorY = yy;
		pgps->NewDataAvailable = 1;
	}
}

/** @brief  : Initial value for GPS functions
**  @agr    : input
**  @retval : Return fix value
**/
void GPS_ParametersInit(GPS *pgps)
{
	pgps->Goal_Flag = Check_NOK;
	pgps->GPS_Error = Veh_NoneError;
	pgps->refPointIndex = -1; // there is no point that was referred
	pgps->K = 0.5;
	pgps->Ksoft = 0.08;
	pgps->Step = 0.5;
}

/** @brief  : GPS updates path yaw 
**  @agr    : GPS
**  @retval : none
**/
void GPS_UpdatePathYaw(GPS *pgps)
{
	for(int i = 0; i < pgps->NbOfP; i++)
	{
		pgps->P_Yaw[i] = atan2(pgps->P_Y[i + 1] - pgps->P_Y[i], pgps->P_X[i + 1] - pgps->P_X[i]);
	}
	pgps->P_Yaw[pgps->NbOfP] = pgps->P_Yaw[pgps->NbOfP - 1];
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
		pflash->Length += ToChar(pgps->P_X[i], &pflash->WriteInBuffer[pflash->Length],6);
		pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
		pflash->Length += ToChar(pgps->P_Y[i], &pflash->WriteInBuffer[pflash->Length],6);
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
double GPS_DMS_To_DD(double LL)
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
double GPS_StringToLat(char *inputmessage)
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
	return GPS_DMS_To_DD(s1 + s2);
}

/** @brief  : Function get lat lon value from GNGLL message
**  @agr    : String value received from message
        @pgps: pointer to instance of GPS
        @inputmessage: a string represent latitude in dddmm.mmmmm format 
**  @retval : Value
**/
double GPS_StringToLng(char *inputmessage)
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
	return GPS_DMS_To_DD(s1 + s2);
}

void GPS_ClearPathBuffer(GPS *pgps)
{
	for(int i = 0; i < MAX_NUM_COORDINATE; ++i)
	{
		pgps->P_X[i] = pgps->P_Y[i] = pgps->P_Yaw[i] = 0;
	}
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
void GPS_StanleyControl(GPS *pgps, double v1_rpm, double v2_rpm)
{
	double dx, dy, d;
	int index = 0, i, upper_bound, lower_bound;
	double v1_mps, v2_mps;
	double L = 0.19, Lf = 0; // L is distance from (Xc, Yc) to the front wheel

	pgps->heading_angle = Pi_To_Pi(-Mag.Angle * (double)pi/180 + pi); // heading angle of vehicle [rad] [-pi, pi]
	v1_mps = Wheel_Radius * 2 * pi * v1_rpm / 60; // [m/s]
	v2_mps = Wheel_Radius * 2 * pi * v2_rpm / 60; // [m/s]
	pgps->Robot_Velocity = (v1_mps + v2_mps)/2;
	pgps->Robot_Velocity = (pgps->Robot_Velocity < 0) ? -pgps->Robot_Velocity : pgps->Robot_Velocity;
	
	// Calculate new Pos if there is no new data from GPS
	if(!pgps->NewDataAvailable)
	{
		pgps->currentPosX += pgps->Robot_Velocity * cos(pgps->heading_angle) * Timer.T;
		pgps->currentPosY += pgps->Robot_Velocity * sin(pgps->heading_angle) * Timer.T;
	}
	// Calculate the front wheel position
	pgps->wheelPosX = pgps->currentPosX + DISTANCE_BETWEEN_GPS_FRONT_WHEEL * cos(pgps->heading_angle);
	pgps->wheelPosY = pgps->currentPosY + DISTANCE_BETWEEN_GPS_FRONT_WHEEL * sin(pgps->heading_angle);

	//Searching the nearest point
	if (pgps->refPointIndex == -1)
	{
		lower_bound = 0;
		upper_bound = pgps->NbOfWayPoints;
	}
	else 
	{
		lower_bound = Max(0, pgps->refPointIndex - SEARCH_OFFSET);
		upper_bound = Min(pgps->NbOfWayPoints, pgps->refPointIndex + SEARCH_OFFSET);
	}

	for(i = lower_bound; i < upper_bound; ++i)
	{
		dx = pgps->wheelPosX - pgps->P_X[i];
		dy = pgps->wheelPosY - pgps->P_Y[i];
		d  = sqrt(dx*dx + dy*dy);

		if(i == lower_bound) 
		{
			pgps->dmin = d;
			index = i;
		}
		else if(pgps->dmin > d) 
		{
			pgps->dmin = d; 
			index = i;	// position of the minimum value
		}
	}

	Lf = pgps->K * pgps->Robot_Velocity + 0.1;
	while((Lf > L) && (index < pgps->NbOfWayPoints - 1))
	{
		dx = pgps->wheelPosX - pgps->P_X[index];
		dy = pgps->wheelPosY - pgps->P_Y[index];
		L = sqrt(dx*dx + dy*dy);
		index++;
	}

	if( index > pgps->refPointIndex )
		pgps->refPointIndex = index;


	pgps->goal_radius = sqrt(pow(pgps->wheelPosX - pgps->P_X[pgps->NbOfWayPoints - 1], 2) + 
						pow(pgps->wheelPosY - pgps->P_Y[pgps->NbOfWayPoints - 1], 2));
		
	if( !VehStt.Veh_Avoid_Flag )
	{
		pgps->efa = -((pgps->wheelPosX - pgps->P_X[pgps->refPointIndex]) * cos(pgps->heading_angle + pi/2) + 
				(pgps->wheelPosY - pgps->P_Y[pgps->refPointIndex]) * sin(pgps->heading_angle + pi/2));

		pgps->Thetae = Pi_To_Pi(pgps->heading_angle - pgps->P_Yaw[pgps->refPointIndex]); // [-pi, pi]
		pgps->Thetad = -atan2( (pgps->K) * (pgps->efa) , (pgps->Robot_Velocity + pgps->Ksoft)); // [-pi, pi]
		pgps->Delta_Angle  = (pgps->Thetae + pgps->Thetad)*(double)180/pi; // [-180, 180]
		if(pgps->Delta_Angle > 160)
			pgps->Delta_Angle = 160;
		else if(pgps->Delta_Angle < -160)
			pgps->Delta_Angle = -160;
	}	
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
void GPS_PursuitControl(GPS *pgps, double v1_rpm, double v2_rpm)
{                   /*   Current pose of the robot   / /  Path coordinate  / /  ThetaP  */
	double dx,dy,d;
	int index = 0, i, upper_bound, lower_bound;
	double Lf, Alpha, v1_mps, v2_mps;
	double L = 0.2, Lfc = 0.6;

	pgps->heading_angle = pi/2 - Mag.Angle * (double)pi/180;
	v1_mps = Wheel_Radius * 2 * pi * v1_rpm/60;	
	v2_mps = Wheel_Radius * 2 * pi * v2_rpm/60;
	pgps->Robot_Velocity = (v1_mps + v2_mps)/2;
	
	// Calculate new Pos if there is no new data from GPS
	if(!pgps->NewDataAvailable)
	{
		pgps->currentPosX += pgps->Robot_Velocity * cos(pgps->heading_angle) * Timer.T;
		pgps->currentPosY += pgps->Robot_Velocity * sin(pgps->heading_angle) * Timer.T;
	}
	// Calculate the back wheel position
	pgps->wheelPosX = pgps->currentPosX - L * cos(pgps->heading_angle);
	pgps->wheelPosY = pgps->currentPosY - L * sin(pgps->heading_angle);

	Lf = pgps->K * pgps->Robot_Velocity + Lfc;
	//Searching the nearest point
	if (pgps->refPointIndex == -1)
	{
		lower_bound = 0;
		upper_bound = pgps->NbOfWayPoints;
	}
	else 
	{
		lower_bound = Max(0, pgps->refPointIndex - SEARCH_OFFSET);
		upper_bound = Min(pgps->NbOfWayPoints, pgps->refPointIndex + SEARCH_OFFSET);
	}

	for(i = lower_bound; i < upper_bound; ++i)
	{
		dx = pgps->wheelPosX - pgps->P_X[i];
		dy = pgps->wheelPosY - pgps->P_Y[i];
		d  = sqrt(dx*dx + dy*dy);

		if(i == lower_bound) 
		{
			pgps->dmin = d;
			index = i;
		}
		else if(pgps->dmin > d) 
		{
			pgps->dmin = d; 
			index = i;	// position of the minimum value
		}
	}

	/* searching look ahead point */
	while((Lf > L) && (index + 1 < pgps->NbOfP))
	{
		dx = pgps->wheelPosX - pgps->P_X[index];
		dy = pgps->wheelPosY - pgps->P_Y[index];
		L = sqrt(pow(dx,2) + pow(dy,2));
		index++;
	}

	if( index > pgps->refPointIndex )
		pgps->refPointIndex = index;

	dx = pgps->wheelPosX - pgps->P_X[pgps->NbOfWayPoints - 1];
	dy = pgps->wheelPosY - pgps->P_Y[pgps->NbOfWayPoints - 1];
	pgps->goal_radius = sqrt(dx*dx + dy*dy);

	if( !VehStt.Veh_Avoid_Flag )
	{
		Alpha = atan2(pgps->wheelPosY - pgps->P_Y[pgps->refPointIndex], 
					pgps->wheelPosX - pgps->P_X[pgps->refPointIndex]) - pgps->heading_angle;
		pgps->Delta_Angle  = atan2(2*L*sin(Alpha), Lf)*180/pi;
	}

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
enum_Error	GPS_NMEA_Message(GPS *pgps, uint8_t *inputmessage,	char result[MESSAGE_ROW][MESSAGE_COL])
{
	int Message_Index = 0;

	while(inputmessage[Message_Index] != '\0' && (Message_Index < ROVER_RX_BUFFERSIZE))
	{
		/* Because GxGGA come first in NMEA */
		if(inputmessage[Message_Index] == (uint8_t)'$')
		{
			if(GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GNGGA"))
			{
#ifdef USE_NMEA_CKSUM
				int Length = LengthOfLine(&inputmessage[Message_Index]);
				if( IsCorrectMessage(&inputmessage[Message_Index + 1], Length - 4, 
					inputmessage[Message_Index + Length - 2], 
					inputmessage[Message_Index + Length - 1]) )
				{
#endif
					GetMessageInfo( (char *)&inputmessage[Message_Index], result, ',');
					if ( (pgps->GPS_Quality = (enum_GPS_Quality) GetValueFromString(&result[6][0])) != 0 )
					{
						GPS_NEO.Latitude = GPS_StringToLat(&result[2][0]); 
						GPS_NEO.Longitude = GPS_StringToLng(&result[4][0]); 
						GPS_LatLonToUTM(&GPS_NEO);
						return Veh_NoneError;
					} 
					else 
						return GPS_DataUnvalid;
#ifdef USE_NMEA_CKSUM
				} 
				else 
					return GPS_GxGGACheckSum_Err;
#endif
			}
		}
		++Message_Index;
	}
	return Veh_NoneError;
}

/*-------------------- Fuzzy control -----------------------*/
/** @brief  : Find maximum number
**  @agr    : 2 input values
**  @retval : Output maximum value
**/
double Fuzzy_Max(double *input,int len)
{
	double max;
	max = input[0];
	for(int i = 1; i < len; i++)
	{
		if(max < input[i])
			max = input[i];
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
	else if(x < ptrimf->a2) result = (x - ptrimf->a1) / (ptrimf->a2 - ptrimf->a1);
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
	Trimf_Update(&In1_NS, -0.22, -0.11, 0.001);
	// ZE : 0 - 0.2
	Trimf_Update(&In1_ZE, -0.11, 0, 0.11);
	// PS : 0.15 - 0.45
	Trimf_Update(&In1_PS, 0.001, 0.11, 0.22);
	// PB : 0.4 - 1
	Trapf_Update(&In1_PB,0.17,0.22,1,2);

	/* Input 2 (edot = Set_thetadot - thetadot) */
	// NE : 0.3 - 1
	Trapf_Update(&In2_NE,-2,-1,-0.4,-0.003);
	// ZE : 0 - 0.4
	Trimf_Update(&In2_ZE, -0.4, 0, 0.4);
	// PO : 0.3 - 1
	Trapf_Update(&In2_PO, 0.003, 0.4, 1, 2);
	/* Output value */
	NB = -0.95;
	NM = -0.8;
	NS = -0.4;
	ZE = 0;
	PS = 0.4;
	PM = 0.8;
	PB = 0.95;
}

void SelectFuzzyOutput(double vel)
{
	/*----------Fuzzy parameter init ------------------*/
	if (vel < MPS2RPM(0.3))
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB : 
		Trapf_Update(&In1_NB, -2,-1, -0.22, -0.17);
		// NS : 
		Trimf_Update(&In1_NS, -0.22, -0.11, -0.044);
		// ZE :
		Trimf_Update(&In1_ZE, -0.056, 0, 0.056);
		// PS :
		Trimf_Update(&In1_PS, 0.044, 0.11, 0.22);
		// PB :
		Trapf_Update(&In1_PB, 0.17, 0.22, 1, 2);

		/* Input 2 (edot = Set_thetadot - thetadot) */
		Trapf_Update(&In2_NE, -2, -1, -0.4, -0.05);
		Trimf_Update(&In2_ZE, -0.4, 0, 0.4);
		Trapf_Update(&In2_PO, 0.05, 0.4, 1, 2);
		/* Output value */
		NB = -0.95;
		NM = -0.8;
		NS = -0.4;
		ZE = 0;
		PS = 0.4;
		PM = 0.8;
		PB = 0.95;
	}
	else
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB :
		Trapf_Update(&In1_NB, -2, -1, -0.45, -0.4);
		// NS :
		Trimf_Update(&In1_NS, -0.45, -0.2, -0.15);
		// ZE :
		Trimf_Update(&In1_ZE, -0.2, 0, 0.2);
		// PS :
		Trimf_Update(&In1_PS, 0.15, 0.2, 0.45);
		// PB : 
		Trapf_Update(&In1_PB, 0.4, 0.45, 1, 2);
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE :
		Trapf_Update(&In2_NE, -2, -1, -0.4, -0.1);
		// ZE : 
		Trimf_Update(&In2_ZE, -0.4, 0, 0.4);
		// PO : 
		Trapf_Update(&In2_PO, 0.1, 0.4, 1, 2);
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
double Defuzzification_Max_Min(double e, double edot)
{
	double pBeta[3], num, den, temp;
	double e_NB, e_NS, e_ZE, e_PS, e_PB, edot_NE, edot_ZE, edot_PO;
	e_NB = Trapf(&In1_NB, e);
	e_NS = Trimf(&In1_NS, e);
	e_ZE = Trimf(&In1_ZE, e);
	e_PS = Trimf(&In1_PS, e);
	e_PB = Trapf(&In1_PB, e);
	edot_NE = Trapf(&In2_NE, edot);
	edot_ZE = Trimf(&In2_ZE, edot);
	edot_PO = Trapf(&In2_PO, edot);
	//NB and NE is NB
	pBeta[0] = Min(e_NB, edot_NE);
	num = NB * pBeta[0];
	den = pBeta[0];
	//NS and NE is NM
	//NB and ZE is NM
	pBeta[0] = Min(e_NS, edot_NE);
	pBeta[1] = Min(e_NB, edot_ZE);
	temp = Fuzzy_Max(pBeta, 2);
	num += NM * temp;
	den += temp;
	//ZE and NE is NS
	//NS and ZE is NS
	//NB and PO is NS
	pBeta[0] = Min(e_ZE, edot_NE);
	pBeta[1] = Min(e_NS, edot_ZE);
	pBeta[2] = Min(e_NB, edot_PO);
	temp = Fuzzy_Max(pBeta, 3);
	num += NS * temp;
	den += temp;
	//PS and NE is ZE
	//ZE and ZE is ZE
	//NS and PO is ZE
	pBeta[0] = Min(e_PS, edot_NE);
	pBeta[1] = Min(e_ZE, edot_ZE);
	pBeta[2] = Min(e_NS, edot_PO);
	temp = Fuzzy_Max(pBeta, 3);
	num += ZE * temp;
	den += temp;
	//PB and NE is PS
	//PS and ZE is PS
	//ZE and PO is PS
	pBeta[0] = Min(e_PB, edot_NE);
	pBeta[1] = Min(e_PS, edot_ZE);
	pBeta[2] = Min(e_ZE, edot_PO);
	temp = Fuzzy_Max(pBeta,3);
	num += PS * temp;
	den += temp;
	//PB and ZE is PM
	//PS and PO is PM
	pBeta[0] = Min(e_PB, edot_ZE);
	pBeta[1] = Min(e_PS, edot_PO);
	temp = Fuzzy_Max(pBeta,2);
	num += PM * temp;
	den += temp;
	//PB and PO is PB
	pBeta[0] = Min(e_PB, edot_PO);
	num += PB * pBeta[0];
	den += pBeta[0];

	return (den == 0) ? 0 : (num/den);
}

double Defuzzification2_Max_Min(double e, double edot) 
{
	double pBeta[10], num, den;
	double e_NB, e_NS, e_ZE, e_PS, e_PB, edot_NE, edot_ZE, edot_PO;
	e_NB = Trapf(&In1_NB, e);
	e_NS = Trimf(&In1_NS, e);
	e_ZE = Trimf(&In1_ZE, e);
	e_PS = Trimf(&In1_PS, e);
	e_PB = Trapf(&In1_PB, e);
	edot_NE = Trapf(&In2_NE, edot);
	edot_ZE = Trimf(&In2_ZE, edot);
	edot_PO = Trapf(&In2_PO, edot);
	if(edot <= -0.4) // u_NE(edot) = 1
	{
		pBeta[0] = e_NB; // u_NB(e) -> output is NB
		pBeta[1] = e_NS; // u_NS(e) -> output is NM
		pBeta[2] = e_ZE; // u_ZE(e) -> output is NS
		pBeta[3] = e_PS; // u_PS(e) -> output is ZE
		pBeta[4] = e_PB; // u_PB(e) -> output is PS
		num = pBeta[0]*NB + pBeta[1]*NM + pBeta[2]*NS + pBeta[3]*ZE + pBeta[4]*PS;
		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4];
	}
	else if (edot < 0) // u_NE(edot) = trapf_NE(edot), u_ZE(edot) = trimf_ZE(edot)
	{
		/* u_NE(edot) = trapf_NE(edot) */
		pBeta[0] = Min(e_NB, edot_NE); // output is NB
		pBeta[1] = Min(e_NS, edot_NE); // output is NM
		pBeta[2] = Min(e_ZE, edot_NE); // output is NS
		pBeta[3] = Min(e_PS, edot_NE); // output is ZE
		pBeta[4] = Min(e_PB, edot_NE); // output is PS
		/* u_ZE(edot) = trimf_ZE(edot) */
		pBeta[5] = Min(e_NB, edot_ZE); // output is NM
		pBeta[6] = Min(e_NS, edot_ZE); // output is NS
		pBeta[7] = Min(e_ZE, edot_ZE); // output is ZE
		pBeta[8] = Min(e_PS, edot_ZE); // output is PS
		pBeta[9] = Min(e_PB, edot_ZE); // output is PM
		num = pBeta[0]*NB + Max(pBeta[1], pBeta[5])*NM + Max(pBeta[2], pBeta[6])*NS + 
				Max(pBeta[3], pBeta[7])*ZE + Max(pBeta[4], pBeta[8])*PS + pBeta[9]*PM;
		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4] + 
				pBeta[5] + pBeta[6] + pBeta[7] + pBeta[8] + pBeta[9];
	}
	else if (edot < 0.4) // u_ZE(edot) = trimf_ZE(edot), u_PO(edot) = trapf_PO(edot)
	{
		/* u_ZE(edot) = trimf_ZE(edot) */
		pBeta[0] = Min(e_NB, edot_ZE); // output is NM
		pBeta[1] = Min(e_NS, edot_ZE); // output is NS
		pBeta[2] = Min(e_ZE, edot_ZE); // output is ZE
		pBeta[3] = Min(e_PS, edot_ZE); // output is PS
		pBeta[4] = Min(e_PB, edot_ZE); // output is PM
		/* u_PO(edot) = trapf_PO(edot) */
		pBeta[5] = Min(e_NB, edot_PO); // output is NS
		pBeta[6] = Min(e_NS, edot_PO); // output is ZE
		pBeta[7] = Min(e_ZE, edot_PO); // output is PS
		pBeta[8] = Min(e_PS, edot_PO); // output is PM
		pBeta[9] = Min(e_PB, edot_PO); // output is PB
		num = pBeta[0]*NM + Max(pBeta[1], pBeta[5])*NS + Max(pBeta[2], pBeta[6])*ZE + 
				Max(pBeta[3], pBeta[7])*PS + Max(pBeta[4], pBeta[8])*PM + pBeta[9]*PB;
		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4] + 
				pBeta[5] + pBeta[6] + pBeta[7] + pBeta[8] + pBeta[9];
	}
	else // u_PO(edot) = 1
	{
		pBeta[0] = e_NB; // u_NB(e) -> output is NS
		pBeta[1] = e_NS; // u_NS(e) -> output is ZE
		pBeta[2] = e_ZE; // u_ZE(e) -> output is PS
		pBeta[3] = e_PS; // u_PS(e) -> output is PM
		pBeta[4] = e_PB; // u_PB(e) -> output is PB
		num = pBeta[0]*NS + pBeta[1]*ZE + pBeta[2]*PS + pBeta[3]*PM + pBeta[4]*PB;
		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4];
	}
	return (den == 0) ? 0 : (num/den);
}


/*------------------------ Flash read/write function ------------------*/

/*------------------------ IMU functions ------------------*/

/** @brief  : Update input for fuzzy controller
**  @agr    : imu and sampletime
**  @retval : none
**/
void	IMU_UpdateFuzzyInput(IMU *pimu)
{
	pimu->Fuzzy_Error = pimu->Set_Angle - pimu->Angle;
	pimu->Fuzzy_Error_dot = -(pimu->Angle - pimu->Pre_Angle) / Timer.T;
	pimu->Pre_Angle = pimu->Angle;
	pimu->Pre_Fuzzy_Out = pimu->Fuzzy_Out;

	if(pimu->Fuzzy_Error > 180) 
		pimu->Fuzzy_Error -= 360;
	else if(pimu->Fuzzy_Error < -180) 
		pimu->Fuzzy_Error += 360;

	pimu->Fuzzy_Error *= pimu->Ke;
	pimu->Fuzzy_Error_dot *= pimu->Kedot;
}

/** @brief  : Update fuzzy coefficients
**  @agr    : imu and Ke,kedot,ku
**  @retval : none
**/
void	IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku)
{
	pimu->Ke	= Ke;
	pimu->Kedot = Kedot;
	pimu->Ku 	= Ku;
}

/** @brief  : Get data from IMU message, update IMU angle
**  @agr    : 
**      @pimu: instance of struct IMU
**      @inputmessage: USART 2 RX BUFFER, contain information receiving from IMU
**          format "0x0A|roll |pitch |yaw |frame index|0x0D"
**          example: "\r| 001960 | 000200 |-000058 |15\n" 
**          unit of angle: mdeg
**  @retval : vehicle error
**/
enum_Error IMU_GetValueFromMessage(IMU *pimu, uint8_t *inputmessage)
{
	int temp = 100000;
	double Angle = 0;
	if(inputmessage[0] == 0x0A && inputmessage[IMU_AngleIndex - 1] == 0x20)
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

