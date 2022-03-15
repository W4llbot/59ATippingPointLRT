#ifndef _8059_PP_SENSORS_HPP_
#define _8059_PP_SENSORS_HPP_
#define voltageToPower 127/12000
extern Node position;
extern double encdS, encdR, bearing, angle;
extern double measuredV, measuredVL, measuredVR;
void sensors(void * ignore);
void setOffset(double i);
#endif
