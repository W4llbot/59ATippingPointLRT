#include "main.h"
double boundRad(double rad){
  double res = fmod(rad, twoPI);
  if(res < 0) res += twoPI;
  return res;
}
double abscap(double x, double abscap){
  if(x > abscap) return abscap;
  else if(x < -abscap) return -abscap;
  else return x;
}
double distance(Node n1, Node n2){
  Node diff = n2 - n1;
  return diff.mag();
}
double circumRad(Node n1, Node n2, Node n3){
  // double a = distance(n1, n2);
  // double b = distance(n2, n3);
  // double c = distance(n3, n1);
  // double denom = sqrt((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c));
  // printf("Square: %.10f\n", ((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c)));
  // if(denom <= INFsmall) return INF;
  // else return a*b*c/denom;

  double x1=n2.getX(), y1=n2.getY(), x2=n1.getX(), y2=n1.getY(), x3=n3.getX(), y3=n3.getY();

  if(x1 == x2) x1 = INFsmall;
  double k1 = 0.5*(pow(x1, 2) + pow(y1, 2) - pow(x2, 2) - pow (y2, 2))/(x1 - x2);
  double k2 = (y1 - y2)/(x1 - x2);

  double b = 0.5*(pow(x2, 2) - 2*x2*k1 + pow(y2, 2) - pow(x3, 2) + 2*x3*k1 - pow(y3, 2))/(x3*k2 - y3 + y2 - x2*k2);
  double a = k1 - k2 * b;

  double r = sqrt(pow((x1 - a), 2) + pow((y1 - b), 2));

  return r;
}
double isEqualDoubles(double a, double b, double epsilon = INFsmall){
  return fabs(a-b) <= epsilon;
}
double bearingToMathAngle(double angle){
  double res = halfPI - angle;
  return boundRad(res);
}

void waitUntil(bool condition) {while(!condition) delay(5);}
void waitUntil(bool condition, double cutoff) {
  double start = millis();
  while(!condition && (millis() - start) < cutoff) delay(5);
}
