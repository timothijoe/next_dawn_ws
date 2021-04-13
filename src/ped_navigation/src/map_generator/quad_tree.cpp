#include "map_generator/quad_tree.h"

quadTree::quadTree()
{
  isleaf = true;
  x = 0;
  y = 0;
  w = 0;
  h = 0;
  depth = 0;
  tree1 = NULL;
  tree2 = NULL;
  tree3 = NULL;
  tree4 = NULL;
}

quadTree::quadTree(int pdepth, double px, double py, double pw, double ph)
{
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

void quadTree::setDim(int pdepth, double px, double py, double pw, double ph)
{
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

void quadTree::get_tree_from_list(vector<single_ped_xvi> z_ped_xvi_list)
{
  vector<single_ped_xvi> ped_xvi_list = z_ped_xvi_list;
  //std::cout << ped_xvi_list.size() << std::endl;
  clear();
  for(auto ped_pos: ped_xvi_list){
    addPoint(ped_pos);
  }
  //std::cout << ped_xvi_list.size() << std::endl;
  //std::cout << "points added!" << std::endl;
}

void quadTree::addChildren()
{
  tree1 = new quadTree(depth + 1, x, y, w/2, h/2);
  tree2 = new quadTree(depth + 1, x + w / 2, y, w / 2, h / 2);
  tree3 = new quadTree(depth + 1, x + w / 2, y + h / 2, w / 2, h / 2);
  tree4 = new quadTree(depth + 1, x, y + h / 2, w / 2, h / 2);
}

void quadTree::clear()
{
  if(isleaf){
    points.clear();
  }
  else{
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

bool quadTree::intersects(double px, double py, double pr) const
{
  if (((px + pr) > x) && ((px - pr) < (x + w)) && ((py + pr) > y) &&
      ((py - pr) < (y + h)))
    return true;  // x+-r/y+-r is inside
  else
    return false;
}

void quadTree::getNeighbors(vector<single_ped_xvi> &neighborlist, double x, double y, double dist)
{
  stack<quadTree*> treestack;
  treestack.push(this);
  while(!treestack.empty()){
    quadTree* t = treestack.top();
    treestack.pop();
    if(t->isleaf){
      t->getPoints(neighborlist);
    }
    else{
      if(t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
      if(t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
      if(t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
      if(t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
    }
  }
}

void quadTree::getPoints(vector<single_ped_xvi> &outputlist)
{
  if(isleaf){
    for(single_ped_xvi currentPoint : points)
      outputlist.push_back(currentPoint);
  }
  else{
    tree1->getPoints(outputlist);
    tree2->getPoints(outputlist);
    tree3->getPoints(outputlist);
    tree4->getPoints(outputlist);
  }
}

void quadTree::addPoint(single_ped_xvi ped)
{
  if(isleaf){
    points.push_back(ped);
  }
  else{
      if((ped.agent_pos[0] >= x + w/2) && (ped.agent_pos[1] >= y + h /2))
        tree3->addPoint(ped);
      else if((ped.agent_pos[0] <= x + w/2) && (ped.agent_pos[1] <= y + h /2))
        tree1->addPoint(ped);
      else if((ped.agent_pos[0] >= x + w/2) && (ped.agent_pos[1] <= y + h /2))
        tree2->addPoint(ped);
      else if((ped.agent_pos[0] <= x + w/2) && (ped.agent_pos[1] >= y + h /2))
        tree4->addPoint(ped);
    }
  if(points.size()>8){
    isleaf = false;
    //std::cout<< "add children" << std::endl;
    addChildren();
    while(!points.empty()){
      single_ped_xvi a = points.back();
      if ((a.agent_pos[0] >= x + w / 2) && (a.agent_pos[1] >= y + h / 2))
        tree3->addPoint(a);  // 3
      else if ((a.agent_pos[0] <= x + w / 2) && (a.agent_pos[1] <= y + h / 2))
        tree1->addPoint(a);  // 1
      else if ((a.agent_pos[0] >= x + w / 2) && (a.agent_pos[1] <= y + h / 2))
        tree2->addPoint(a);  // 2
      else if ((a.agent_pos[0] <= x + w / 2) && (a.agent_pos[1] >= y + h / 2))
        tree4->addPoint(a);  // 4
      points.pop_back();
    }
  }
}
