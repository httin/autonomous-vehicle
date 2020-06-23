#include "TwoWheelRobot.h"
#include "functions.h"
	
SelfPosition selfPosition;

void Self_ParametersInit(SelfPosition *pself)
{
	pself->R  = Wheel_Radius;
}

void SelfPositionUpdateParams(SelfPosition *selfPos, double rpm_left, double rpm_right, double yaw, double sampleTime)
{
    //double vlinear_left, vlinear_right;
    double v_dir;
    /* 
     * Calculate velocity: 
     * linear_velocity[m/s] = R * angular_velocity[rad/s] = R * (RPM * 2pi/60) 
     */
    selfPos->w_left = rpm_left * 0.10472; 
    selfPos->w_right = rpm_right * 0.10472;
    //vlinear_left = selfPos->R * selfPos->w_left;
    //vlinear_right = selfPos->R * selfPos->w_right;
    v_dir = (selfPos->w_left + selfPos->w_right) * selfPos->R / 2;
    yaw = Pi_To_Pi(pi/2 - yaw*pi/180);
	// Update new position
    selfPos->x = selfPos->x + v_dir * cos(yaw) * sampleTime; // x' = x + v*t*cos(yaw)
    selfPos->y = selfPos->y + v_dir * sin(yaw) * sampleTime; // y' = y + v*t*sin(yaw)
}

void OverWritePosition(SelfPosition *selfPos, double x, double y)
{
    selfPos->x = x;
    selfPos->y = y;
}

