#include "main.h"
Node::Node(): x{0}, y{0}
{}
Node::Node(double p_x, double p_y): x{p_x}, y{p_y}
{}
Node operator+(const Node &n1, const Node &n2){
  return {n1.x + n2.x, n1.y + n2.y};
}
Node operator-(const Node &n1, const Node &n2){
  return {n1.x - n2.x, n1.y - n2.y};
}
Node operator*(const Node &n, double val){
  return {n.x*val, n.y*val};
}
Node operator/(const Node &n, double val){
  return {n.x/val, n.y/val};
}
double Node::getX(){
  return x;
}
double Node::getY(){
  return y;
}
void Node::setXY(double x, double y){
  this -> x = x;
  this -> y = y;
}
void Node::print(){
  printf("{%.2f, %.2f}", x, y);
}
double Node::mag(){
  return sqrt(pow(this-> x, 2) + pow(this-> y, 2));
}
double Node::TDot(Node n){
  // transpose and then dot product
  return x * n.x + y * n.y;
}
std::vector<double> Node::findLookAhead(Node start, Node end, double lookAheadDist){
  std::vector<double> res(2, 0);
  /**
   * res[0]= 0 if there's no intersection, 1 if t1 or t2 is chosen
   * res[1] is either t1 or t2 depending on case
   * */
  Node d = end - start;
  Node f = start - *this;
  double a = d.TDot(d);
  double b = 2*(f.TDot(d));
  double c = f.TDot(f) - lookAheadDist*lookAheadDist;
  double disc = b*b - 4*a*c; //discriminant
  if(disc < 0){
    res[0] = 0; //safety
    return res;
  }else{
    disc = sqrt(disc);
    double t1 = (-b-disc)/(2*a);
    double t2 = (-b+disc)/(2*a);
    if(t1>=0 && t1 <= 1){
      res[0] = 1; //true flag
      res[1] = t1;
      return res;
    }
    if(t2>=0 && t2<=1){
      res[0] = 1;
      res[1] = t2;
      return res;
    }
    //otherwise there's no intersection
    res[0] = 0;
    return res;
  }

}
Node Node::norm(){
  double mag = this->mag();
  return {(this->x)/mag, (this->y)/mag};
}
