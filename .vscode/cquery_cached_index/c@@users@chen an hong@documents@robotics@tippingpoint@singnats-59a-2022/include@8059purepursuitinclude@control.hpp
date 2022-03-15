#ifndef _8059_PP_CONTROL_HPP_
#define _8059_PP_CONTROL_HPP_
#define TURN_LEEWAY 2
#define LEEWAY 3
#define DEFAULT_W_SMOOTH 0.75
#define DEFAULT_W_DATA 1-DEFAULT_W_SMOOTH
#define DEFAULT_LOOK_AHEAD 24
#define dT 5
extern bool enablePP;

void park(double s);
void drive(double l, double r);
void enableBase(bool left, bool right);
void baseTurn(double p_bearing, double kp);
void baseTurn(double p_bearing);
double calcBaseTurn(double x, double y, bool rev);
void baseMove(double dis);
void baseMove(double x, double y);
void baseMove(double x, double y, bool rev);
void waitTurn(double cutoff);
void resetPP();
void basePP(std::vector<Node> wps, double p_w_data = DEFAULT_W_DATA, double p_w_smooth = DEFAULT_W_SMOOTH, double p_lookAhead = DEFAULT_LOOK_AHEAD, bool p_reverse = false);
void waitPP(double cutoff);
void PPControl(void * ignore);
#endif
