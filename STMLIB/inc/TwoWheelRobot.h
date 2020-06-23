#ifndef TWO_WHEEL_ROBOT_H
#define TWO_WHEEL_ROBOT_H

#include <math.h>

typedef struct SelfPosition
{
    double x; 
    double y;
    double w_left;
    double w_right;
    double R;
} SelfPosition;

void Self_ParametersInit(SelfPosition *pself);
void SelfPositionUpdateParams(SelfPosition *selfPos, double rpm_left, double rpm_right, double yaw, double sampleTime);
void OverWritePosition(SelfPosition *selfPos, double x, double y);

extern SelfPosition selfPosition;

#endif 
