#ifndef _quad_tree_collision_checking_h_
#define _quad_tree_collision_checking_h_ 1

#include <set>
#include <vector>

using namespace std;

template<class T>
class Quadtree{

 public:
  Quadtree(int depth, double x, double y, double w, double h);
  ~Quadtree();

  void clear();
  void addAgent(const T & a);
  void addAgentList(std::vector<T>& agentList);
  bool intersects(double px, double py, double pr) const;


 protected:

  bool isleaf;
  double x;
  double y;
  double w;
  double h;
  int depth;
  std::vector<T> agents;

  Quadtree<T>* tree1;
  Quadtree<T>* tree2;
  Quadtree<T>* tree3;
  Quadtree<T>* tree4;
};


Quadtree<T>::Quadtree(int pdepth, double px, double py,
                  double pw, double ph){
  isleaf = true;
  x = px;
  y = py;
  w = pw;
  h = ph;
  depth = pdepth;
  tree1 = NULL;
  tree2 = NULL;
  tree3 = NULL;
  tree4 = NULL;
}

void addAgentList(const std::vector<T>& agentList){
  for(auto iter = agentList.begin(); iter != agent.end(); iter++){
    addAgent((*iter));
  }
}
void Quadtree::clear() {
  if (isleaf) {
    agents.clear();
  } else {
    tree1->clear();
    delete tree1;
    tree2->clear();
    delete tree2;
    tree3->clear();
    delete tree3;
    tree4->clear();
    delete tree4;
    isleaf = true;
  }
}

void Quadtree::addChildren() {
  tree1 = new Ped::Ttree(depth + 1, x, y, w / 2, h / 2);
  tree2 = new Ped::Ttree(depth + 1, x + w / 2, y, w / 2, h / 2);
  tree3 = new Ped::Ttree(depth + 1, x + w / 2, y + h / 2, w / 2, h / 2);
  tree4 = new Ped::Ttree(depth + 1, x, y + h / 2, w / 2, h / 2);
}

void Quadtree::addAgent(const T a) {
  if (isleaf) {
    agents.push_back(a);
  } else {
    if ((a.x >= x + w / 2) && (a.y >= y + h / 2))
      tree3->addAgent(a);  // 3
    if ((a.x <= x + w / 2) && (a.y <= y + h / 2))
      tree1->addAgent(a);  // 1
    if ((a.x >= x + w / 2) && (a.y <= y + h / 2))
      tree2->addAgent(a);  // 2
    if ((a.x <= x + w / 2) && (a.y >= y + h / 2))
      tree4->addAgent(a);  // 4
  }

  if (agents.size() > 8) {
    isleaf = false;
    addChildren();
    while (!agents.empty()) {
      T a = (*agents.begin());
      if ((a.x >= x + w / 2) && (a.y >= y + h / 2))
        tree3->addAgent(a);  // 3
      if ((a.x <= x + w / 2) && (a.y <= y + h / 2))
        tree1->addAgent(a);  // 1
      if ((a.x >= x + w / 2) && (a.y <= y + h / 2))
        tree2->addAgent(a);  // 2
      if ((a.x <= x + w / 2) && (a.y >= y + h / 2))
        tree4->addAgent(a);  // 4
      agents.erase(agents.begin());
    }
  }
}

void Quadtree::getAgents(std::vector<T>& outputList) {
  if(isleaf){
    for(auto iter=agents.begin(); iter != agents.end(), iter++)
    {
      neighborList.push_back(*iter);
    }
  else{
    tree1->getAgents(outputList);
    tree2->getAgents(outputList);
    tree3->getAgents(outputList);
    tree4->getAgents(outputList);
  }
  }
}

int Quadtree::cut() {
  if (isleaf) return agents.size();

  int count = 0;
  count += tree1->cut();
  count += tree2->cut();
  count += tree3->cut();
  count += tree4->cut();
  if (count < 5) {
    assert(tree1->isleaf == true);
    assert(tree2->isleaf == true);
    assert(tree3->isleaf == true);
    assert(tree4->isleaf == true);
    agents.push_back(tree1->agents.begin(), tree1->agents.end());
    agents.push_back(tree2->agents.begin(), tree2->agents.end());
    agents.push_back(tree3->agents.begin(), tree3->agents.end());
    agents.push_back(tree4->agents.begin(), tree4->agents.end());
    isleaf = true;
    delete tree1;
    delete tree2;
    delete tree3;
    delete tree4;
  }
  return count;
}



bool Quadtree::intersects(double px, double py, double pr) const {
  if (((px + pr) > x) && ((px - pr) < (x + w)) && ((py + pr) > y) &&
      ((py - pr) < (y + h)))
    return true;  // x+-r/y+-r is inside
  else
    return false;
}

void Quadtree::getNeighbors(vector<T>& neighborList,
                               double x, double y, double dist) const {
  stack<Quadtree*> treestack;

  treestack.push(tree);
  while (!treestack.empty()) {
    Quadtree* t = treestack.top();
    treestack.pop();
    if (t->isleaf) {
      t->getAgents(neighborList);
    } else {
      if (t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
      if (t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
      if (t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
      if (t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
    }
  }
}

bool Quadtree::fine_collision_checking(const vector<T>& neighborList, double x, double y, double dist)
{
  for(auto iter = neighborList.begin(); iter != neighborList.end(); iter++){
    double d_result = sqrt(((*iter).x - x)^2 + ((*iter).x - x)^2);
    if(d_result <= dist){
      std::cout << "collision happens, it have collisions with "<< static_cast<int>(*iter).agent_id << "-th pedestrian";
    }
    return true;
  }
  return false;
}
#endif
