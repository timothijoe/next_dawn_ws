#ifndef QUAD_TREE_H
#define QUAD_TREE_H

#include <Eigen/Eigen>
#include <set>
#include <vector>
#include <map_generator/ped_xvi.h>

#include <stack>
using namespace std;
using namespace Eigen;

class quadTree
{
public:
  quadTree();
  quadTree(int depth, double px, double py, double pw, double ph);
  void setDim(int depth, double px, double py, double pw, double ph);
  void get_tree_from_list(vector<single_ped_xvi> z_ped_xvi_list);
  void addChildren();
  void clear();
  bool intersects(double px, double py, double pr) const;
  void getNeighbors(vector<single_ped_xvi> &neighborlist, double x, double y, double dist);
  void getPoints(vector<single_ped_xvi> &outputlist);
  void addPoint(single_ped_xvi ped);
protected:
  vector<single_ped_xvi> points;
  bool isleaf;
  double x;
  double y;
  double w;
  double h;
  int depth;

  quadTree* tree1;
  quadTree* tree2;
  quadTree* tree3;
  quadTree* tree4;
};

#endif // QUAD_TREE_H
