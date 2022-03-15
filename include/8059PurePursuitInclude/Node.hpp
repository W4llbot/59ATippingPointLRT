#ifndef _8059_PP_NODE_HPP_
#define _8059_PP_NODE_HPP_
#include <vector>
class Node{
private:
  double x, y;
public:
  Node();
  Node(double p_x, double p_y);
  friend Node operator+(const Node &n1, const Node &n2);
  friend Node operator-(const Node &n1, const Node &n2);
  friend Node operator*(const Node &n, double val);
  friend Node operator/(const Node &n, double val);
  double getX();
  double getY();
  void setXY(double x, double y);
  void print();
  double mag();
  double TDot(Node n);
  std::vector<double> findLookAhead(Node start, Node end, double lookAheadDist);
  Node norm();
};
class Node;
#endif
