#include "main.h"
/** to test odometry in opcontrol() when not in competition */
#define COMPETITION_MODE false
/** Update the robot's position using side encoders values. */
void Odometry(void * ignore){
  Controller master(E_CONTROLLER_MASTER);

  /** D loop variables */
  double prevEncdR = 0, prevEncdS = 0, prevBearing = bearing;
  int count = 0;
  while(!COMPETITION_MODE || competition::is_autonomous()){
    // Amount moved by robot
    double encdChangeR = encdR-prevEncdR;
    double encdChangeS = encdS-prevEncdS;
    double bearingChange = bearing - prevBearing;

    // Update prev variables
		prevEncdR = encdR;
		prevEncdS = encdS;
    prevBearing = bearing;

    // Calculate local offset
    Node localOffset;
    if(bearingChange) {
      localOffset = Node(encdChangeS/bearingChange + S_DIS, encdChangeR/bearingChange + R_DIS) * 2*sin(bearingChange/2);
    }else {
      localOffset = Node(encdChangeS, encdChangeR);
    }

    // Calculate global offset rotated by avg rotation
    double avgBearing = prevBearing + bearingChange/2;
    Node rotatedOffset = Node(cos(-avgBearing)*localOffset.getX() - sin(-avgBearing)*localOffset.getY(),
                              sin(-avgBearing)*localOffset.getX() + cos(-avgBearing)*localOffset.getY());
    position = position + rotatedOffset;

    // if(count++ % 100 == 0) printf("X: %.2f, Y: %.2f, bearing: %.2f\n", position.getX(), position.getY(), bearing/toRad);
    
    
    master.print(2, 0, "%.2f, %.2f, %.2f          ", position.getX(), position.getY(), bearing/toRad);
    
    
    // printf("sDis: %.5f, sBearing: %.5f\t", sDis, sBearing/toRad);
    // printf("dX: %.2f, dY: %.5f\n", deltaX, deltaY);


    /** debugging */
    // if(!COMPETITION_MODE) posPrintMaster();

    // if(DEBUG_MODE == 4) encdPrintTerminal();
    /** refresh rate of Task */
    Task::delay(dT);
  }
}
