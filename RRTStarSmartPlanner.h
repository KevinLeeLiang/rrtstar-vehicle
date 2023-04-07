//
// Created by garen-lee on 2023/4/4.
//

#ifndef RRTSTARTSMART_VEHICLE_RRTSTARSMARTPLANNER_H
#define RRTSTARTSMART_VEHICLE_RRTSTARSMARTPLANNER_H

#include <iostream>
#include <random>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

// 车辆状态类
class State {
public:
  double x;
  double y;
  double theta;
  double v;

  State(double x_, double y_, double theta_, double v_) : x(x_), y(y_), theta(theta_), v(v_) {}
};

// 树节点类
class Node {
public:
  State* state;
  Node* parent;
  double cost;

  Node(State* state_) : state(state_), parent(nullptr), cost(0.0) {}
};

// RRT*-Smart算法类
class RRTStarSmartPlanner {
public:
  vector<Node *> tree;  // 树
  double goal_max_distance;  // 目标最大距离
  double neighbor_radius;  // 邻域半径
  double speed_max;  // 最大速度
  double L;  // 轴距
  double dt;  // 时间间隔
public:
  RRTStarSmartPlanner(double goal_max_distance_, double neighbor_radius_, double speed_max_, double L_, double dt_); //构造函数，初始化参数。
  void addStartNode(const State& start_state);  //添加初始节点。
  void addGoalNode(const State& goal_state);    //添加目标节点。
  Node* getRandomNode();                        //获取随机节点。
  Node* getNearestNode(vector<Node*>& nodes, const State& point);         //获取距离point最近的节点。
  Node* getNewNode(Node* nearest_node, const State& rand_state);          //创建新节点。
  bool isCollisionFree(Node* node1, Node* node2);                         //判断两个节点之间是否与障碍物相撞。
  vector<Node*> findNearNodes(vector<Node*>& tree, const State& point);   //查找在邻域半径范围内的节点。
  void rewire(Node* new_node, vector<Node*>& near_nodes);                 //尝试重新连接near_nodes中的每个节点。
  double getCost(const State& state1, const State& state2);               //计算state1和state2之间的平面距离。
  double getVelocity(const State* state1, const State& state2);           //计算车辆在state1处到达state2时的最大速度。
  vector<State> getPath(Node* goal_node);                                 //从根节点到目标节点返回一条路径。
}


#endif //RRTSTARTSMART_VEHICLE_RRTSTARSMARTPLANNER_H
