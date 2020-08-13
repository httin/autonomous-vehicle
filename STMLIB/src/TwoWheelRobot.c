#include "TwoWheelRobot.h"
#include "functions.h"

SelfPosition selfPosition;

void SelfPositionUpdateParams(SelfPosition *selfPos, double rpm_left, double rpm_right, double yaw, double sampleTime)
{
    double v_linear;
    /*
     * Convert rpm to rad/s: 1 [RPM] = 0.10472 [rad/sec]
     */
    selfPos->w_left = rpm_left * 0.10472; 
    selfPos->w_right = rpm_right * 0.10472;
    /* 
     * Calculate velocity: 
     * linear_velocity[m/s] = R * angular_velocity[rad/s] = R * (RPM * 2pi/60) 
     */
    v_linear = (selfPos->w_left + selfPos->w_right) * selfPos->R / 2;
    yaw = Pi_To_Pi(pi - yaw*(double)pi/180);
	// Update new position
    selfPos->x = selfPos->x + v_linear * cos(yaw) * sampleTime; // x' = x + v*t*cos(yaw)
    selfPos->y = selfPos->y + v_linear * sin(yaw) * sampleTime; // y' = y + v*t*sin(yaw)
}

void updateSelfPos(SelfPosition *selfPos, double x, double y)
{
    selfPos->x = x;
    selfPos->y = y;
}

