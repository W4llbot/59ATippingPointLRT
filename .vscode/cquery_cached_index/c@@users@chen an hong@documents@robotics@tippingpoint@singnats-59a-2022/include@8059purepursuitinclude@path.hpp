#ifndef _8059_PP_PATH_HPP_
#define _8059_PP_PATH_HPP_
#include "Node.hpp"
#include <vector>
#define inPerDeg 0.0240091045675568 //use baseOdom to tune

// Motor 600 RPM
// Gear Ratio 3:5
#define RPMToInPerMs 1/60/1000*360*inPerDeg
#define inPerMsToRPM 1/inPerDeg*1000*60/360
// #define MAXRPMV 500.0
// #define MAXRPMA 1.0
//
#define voltageToPower 127/12000
// #define globalMaxV MAXRPMV * RPMToInPerMs
// #define globalMaxA MAXRPMA * RPMToInPerMs
extern double globalMaxV, globalMaxA;

//INJECT
#define SPACING 1
//SMOOTH
#define TOLERANCE 0.001
//MAXV
// #define K 0.017
// #define K 0.0000000000000004
// #define K 1

void setMaxRPMV(double rpm);
class Path{
private:
  std::vector<Node> wps;
  std::vector<Node> injWps;

  std::vector<Node> smoWps;
  double w_data, w_smooth, lookAhead;
  int n;

  std::vector<double> dist;
  std::vector<double> curv;
  std::vector<double> maxV;
  std::vector<double> targV;

public:
  Path();
  Path(std::vector<Node> p_wps);
  Node getSmoWp(int i);
  double getMaxV(int i);
  double getTargV(int i);
  int getN();
  double getLookAhead();
  void inject();
  void smooth();
  void calcDist();
  void calcCurvature();
  void calcMaxV();
  void calcTargV();
  void setWps(std::vector<Node> p_wps, double p_w_data, double p_w_smooth, double p_lookAhead);
  void debugPoint(int i);
};

void setCurvK(double d);

#define debugPath true

#endif
