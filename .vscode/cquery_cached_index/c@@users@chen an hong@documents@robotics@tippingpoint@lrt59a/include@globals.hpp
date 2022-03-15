#ifndef _GLOBALS_HPP_
#define _GLOBALS_HPP_

// Motor ports
#define FLPort 8
#define BLUPort 9
#define BLDPort 10
#define FRPort 18
#define BRUPort 17
#define BRDPort 16

// Mech Ports
#define armPort 1
#define intakePort 2

// Pneumatic Ports
#define tiltPort 6
#define tiltClampPort {{7, 8}}
#define armClampPort {{7, 4}}

// Sensor ports
#define armLimitPort {7, 5}
#define tiltLimitPort {7, 6}
#define potentiometerPort 1
#define encdRPort 7
//{SMART_PORT, PORT_TOP, PORT_BOTTOM}
#define encdSPort {7,1,2}
#define imuPort 11

#endif
