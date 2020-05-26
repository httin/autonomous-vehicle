#include "TwoWheelRobot.h"
#include "functions.h"
#define				pi				(double)3.14159265358979
	
SelfPosition selfPosition;

void Self_ParametersInit(SelfPosition *pself)
{
	pself->R  = 0.083;
	pself->x  = 0;
	pself->y  = 0;
}

void SelfPositionUpdateParams(SelfPosition *selfPos, double w_left, double w_right, double yaw, double sampleTime)
{
    double v_left, v_right, v_dir;
    // Calculate velocity with multiple directions
    v_left = selfPos->R * w_left * 0.10472;
    v_right = selfPos->R * w_right * 0.10472;
    v_dir = (v_right + v_left) / 2;
    yaw = Pi_To_Pi(pi/2 - yaw*pi/180);
	// Update new position
    selfPos->x = selfPos->x + v_dir * cos(yaw) * sampleTime;
    selfPos->y = selfPos->y + v_dir * sin(yaw) * sampleTime;
}

void OverWritePosition(SelfPosition *selfPos, double x, double y)
{
    selfPos->x = x;
    selfPos->y = y;
}




