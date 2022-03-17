#include "main.h"

double k = 0.017;

double maxRPMV = 500.0;
double maxRPMA = 1.0;

double globalMaxV = maxRPMV * RPMToInPerMs;
double globalMaxA = maxRPMA * RPMToInPerMs;

void setMaxRPMV(double rpm) {
  maxRPMV = rpm;
  globalMaxV = maxRPMV * RPMToInPerMs;
}

Path::Path(): wps{}
{}
Path::Path(std::vector<Node> p_wps): wps{p_wps}
{}
Node Path::getSmoWp(int i){
  return smoWps[i];
}
double Path::getMaxV(int i){
  return maxV[i];
}
double Path::getTargV(int i){
  return targV[i];
}
int Path::getN(){
  return n;
}
double Path::getLookAhead(){
  return lookAhead;
}
void Path::inject(){
  injWps.clear();
  for(int i = 0;i<=wps.size()-2;++i){
    Node start = wps[i];
    Node end = wps[i+1];
    Node diff = end - start;
    Node step = diff.norm()*SPACING;
    int nFit = ceil(diff.mag()/SPACING);
    for(int j = 0;j<nFit;++j){
      injWps.push_back(start + step*j);
    }
  }
  injWps.push_back(wps[wps.size()-1]);
  n = injWps.size();
}
void Path::smooth(){
  smoWps = injWps;
  double change = TOLERANCE;
  while(change >= TOLERANCE){
    change = 0;
    for(int i = 1;i<n-1;++i){
      Node aux = smoWps[i];
      smoWps[i] = aux + (injWps[i] - smoWps[i])*w_data + (smoWps[i-1] + smoWps[i+1] - smoWps[i]*2)*w_smooth;

      Node diff = smoWps[i] - aux;
      change += (fabs(diff.getX()) + fabs(diff.getY()));
    }
    // printf("change = %.5f\n", change);
  }
}
void Path::calcDist(){
  Node prevWp = smoWps[0];
  double prevDist = 0;
  for(auto smoWp : smoWps){
    double deltaDist = distance(smoWp, prevWp);
    double newDist = prevDist + deltaDist;
    dist.push_back(newDist);
    prevDist = newDist;
    prevWp = smoWp;
  }
  // printVector(dist);
}
void Path::calcCurvature(){
  curv.push_back(0); //starting point
  for (int i = 1;i<=n-2;++i){
    double r = circumRad(smoWps[i-1], smoWps[i], smoWps[i+1]);
    curv.push_back(1/r);
  }
  // printVector(curv);
  curv.push_back(0); //final point
}
void Path::calcMaxV(){
  for(int i = 0;i<n;++i){
    // printf("MaxV: %.5f, Curve: %.5f\n", globalMaxV, K/curv[i]);
    maxV.push_back(std::min(globalMaxV, k/curv[i]));
  }
  // printVector(maxV);
}
void Path::calcTargV(){
  targV.clear();
  targV.resize(n, 0);
  targV[n-1] = 0;
  for(int i = n-2;i>=0;--i){
    double d = distance(smoWps[i+1], smoWps[i]);
    targV[i] = std::min(maxV[i], sqrt(targV[i+1]*targV[i+1] + 2 * globalMaxA * d));
  }
}
void Path::setWps(std::vector<Node> p_wps, double p_w_data, double p_w_smooth, double p_lookAhead){
  //perform injection and smooth
  wps = p_wps;
  w_data = p_w_data;
  w_smooth = p_w_smooth;
  lookAhead = p_lookAhead;
  this->inject();
  this->smooth();
  this->calcDist();
  this->calcCurvature();
  this->calcMaxV();
  this->calcTargV();

  enablePP = true;
  if (debugPath){
    printf("Path: \n");
    for(int i = 0; i < n; i++) {
      debugPoint(i);
      printf("\n");
    }
  }
}

void Path::debugPoint(int i) {
  printf("{%.2f, %.2f}\t", smoWps[i].getX(), smoWps[i].getY());
  printf("Dist: %.2f\t", dist[i]);
  printf("Curv: %.20f\t", curv[i]);
  printf("MaxV: %.5f\t", maxV[i]*inPerMsToRPM);
  printf("TargV: %.5f\t", targV[i]*inPerMsToRPM);
  // printf("\n");
}

void setCurvK(double d) {
  k = d;
}
// void setMaxRPM(double rpm) {
//   MAXRPMV = rpm;
// }
