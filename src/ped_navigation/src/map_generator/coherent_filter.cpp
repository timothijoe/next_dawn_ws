#include "map_generator/coherent_filter.h"

coherentFilter::coherentFilter()
{

}

void coherentFilter::reset_cur_peds(vector<single_ped_xvi> z_cur_peds_xvi)
{
  cur_peds_xvi.clear();
  cur_peds_xvi = z_cur_peds_xvi;
}

void coherentFilter::get_neighbor()
{
  _neighbor_dict.clear();
  for(auto it = cur_peds_xvi.begin(); it != cur_peds_xvi.end(); ++it){
    certain_ped_neighbor certain_neighbor = certain_ped_neighbor(*it);
    vector<int> cur_peds_sorted = certain_neighbor.sort(cur_peds_xvi);
    _neighbor_dict[certain_neighbor.ego_id] = cur_peds_sorted;
  }
}

void coherentFilter::pair_to_cluster()
{
  _cluster_index.clear();
  _cluster_inverse_index.clear();
  vector<Vector2i> pair_list;

  // Taking coherent neighbor of a certain person to pairs
  for (auto it=_neighbor_dict.begin(); it!=_neighbor_dict.end(); ++it){
    for(auto ele = it->second.begin(); ele != it->second.end(); ++ele){
      Vector2i pair;
      pair << it->first, *ele;
      pair_list.push_back(pair);
    }
  }

  // Initialize the neighbor
  for(auto it = cur_peds_xvi.begin(); it != cur_peds_xvi.end(); ++it){
    _cluster_index[it->agent_id] = 0;
  }
  int clusterNum = 0;
  int pair_list_size = pair_list.size();
  for(int num = 0; num < pair_list_size; num++){
    //std::cout <<pair_list[num] << std::endl;
    Vector2i pair = pair_list[num];
    int curPairAlabel = _cluster_index[pair[0]];
    int curPairBlabel = _cluster_index[pair[1]];
    // Both A and B are not clustered
    if((curPairAlabel == 0) && (curPairBlabel == 0)){
      clusterNum = clusterNum + 1;
      int curPairLabel = clusterNum;
      _cluster_index[pair[0]] = curPairLabel;
      _cluster_index[pair[1]] = curPairLabel;
      vector<int> list;
      list.push_back(pair[0]);
      list.push_back(pair[1]);
      _cluster_inverse_index[curPairLabel] = list;
    }
    // if A is clustered and B is not
    else if((curPairAlabel != 0) && (curPairBlabel == 0)){
      _cluster_index[1] = curPairAlabel;
      _cluster_inverse_index[curPairAlabel].push_back(pair[1]);
    }
    // if B is clustered and A is not
    else if((curPairAlabel == 0) && (curPairBlabel != 0)){
      _cluster_index[0] = curPairBlabel;
      _cluster_inverse_index[curPairBlabel].push_back(pair[0]);
    }
    // if both are not 0 and they are equal
    else if(curPairAlabel == curPairBlabel){
      continue;
    }
    // if curPairAlabel < curPairBlabel and both of them are not 0
    else if(curPairAlabel < curPairBlabel){
      for(auto mem_id : _cluster_inverse_index[curPairBlabel]){
        _cluster_index[mem_id] = curPairAlabel;
        _cluster_inverse_index[curPairAlabel].push_back(mem_id);
      }
      _cluster_inverse_index.erase(curPairBlabel);
    }
    // if curPairAlabel > curPairBlabel and both of them are not 0
    else{
      for(auto mem_id : _cluster_inverse_index[curPairAlabel]){
        _cluster_index[mem_id] = curPairBlabel;
        _cluster_inverse_index[curPairBlabel].push_back(mem_id);
      }
      _cluster_inverse_index.erase(curPairAlabel);
    }

  }
}

void coherentFilter::reidentify()
{
  re_cluster_index.clear();
  int newClusterNum = 0;
  for(auto cluster: _cluster_inverse_index){
    newClusterNum += 1;
    for(auto mem: cluster.second){
      re_cluster_index[mem] = newClusterNum;
    }
  }
}

map<int, int> coherentFilter::get_cluster(vector<single_ped_xvi> z_cur_peds_xvi)
{
  reset_cur_peds(z_cur_peds_xvi);
  get_neighbor();
  pair_to_cluster();
  reidentify();
//  std::cout << "cluster size: "<< _cluster_inverse_index.size() << std::endl;
//  for(auto cluster_index:re_cluster_index){
//    std::cout<< cluster_index.first << " : " << cluster_index.second << std::endl;
//  }
  return re_cluster_index;
}
