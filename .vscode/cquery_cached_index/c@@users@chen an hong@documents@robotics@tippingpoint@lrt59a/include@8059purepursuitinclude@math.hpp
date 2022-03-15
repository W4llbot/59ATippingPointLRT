#ifndef _8059_PP_MATH_HPP_
#define _8059_PP_MATH_HPP_
#include "Node.hpp"
#define INF 1e5
#define INFsmall 1/INF
#define PI      3.14159265358979323846264338328
#define halfPI  1.57079632679489661923132169164
#define twoPI	  6.28318530717958647692528676656
#define toDeg   57.2957795130823208767981548141
#define toRad   0.0174532925199432957692369076849

double boundRad(double rad);
double abscap(double x, double abscap);
double distance(Node n1, Node n2);
double circumRad(Node n1, Node n2, Node n3);
double isEqualDoubles(double a, double b, double epsilon);
double bearingToMathAngle(double angle);

void waitUntil(bool condition);
void waitUntil(bool condition, double cutoff);
#endif
