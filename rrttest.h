//
// Created by garen-lee on 2023/4/7.
//

#ifndef RRTSTARTSMART_VEHICLE_RRTTEST_H
#define RRTSTARTSMART_VEHICLE_RRTTEST_H
#include <iostream>

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
// 定义节点结构体
struct Node {
  double x;
  double y;
  double theta;
  double cost;
  Node* parent;
};

// 定义全局变量
const double PI = 3.14159265358979323846;
const double STEER_ANGLE_MAX = 0.5236; // 最大转角（30度）
const double VEL_MAX = 5.0; // 最大速度
const double DT = 0.1; // 时间步长
const double GOAL_TOL = 5; // 目标误差允许范围
const double MAP_WIDTH = 50.0; // 地图宽度
const double MAP_HEIGHT = 50.0; // 地图高度
const double OBSTACLE_RADIUS = 2.0; // 障碍物半径

// 定义随机数生成器
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<> dis_x(0.0, MAP_WIDTH);
uniform_real_distribution<> dis_y(0.0, MAP_HEIGHT);

// 计算两个点之间的欧几里得距离
double distance(Node* n1, Node* n2) {
  return sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));
}

// 计算两个角度之间的最小差值
double angleDiff(double a1, double a2) {
  double diff = fmod(a2 - a1 + PI, 2 * PI) - PI;
  if (diff < -PI) {
    diff += 2 * PI;
  }
  return diff;
}

// 判断两个节点之间是否存在障碍物
bool isCollision(Node* n1, Node* n2, vector<Node*>& obstacles) {
  double d = distance(n1, n2);
  for (auto& obstacle : obstacles) {
    double d1 = distance(n1, obstacle);
    double d2 = distance(n2, obstacle);
    if (d1 <= OBSTACLE_RADIUS || d2 <= OBSTACLE_RADIUS) {
      return true;
    }
    double x3 = obstacle->x;
    double y3 = obstacle->y;
    double a1 = atan2(n2->y - n1->y, n2->x - n1->x);
    double a2 = atan2(y3 - n1->y, x3 - n1->x);
    double da = angleDiff(a1, a2);
    double d3 = abs(d1 * sin(da));
    if (d3 <= OBSTACLE_RADIUS && d1 * cos(da) >= 0 && d2 * cos(da) <= 0) {
      return true;
    }
  }
  return false;
}

// 生成随机点
Node* getRandomNode() {
  Node* node = new Node();
  node->x = dis_x(gen);
  node->y = dis_y(gen);
  node->theta = dis_x(gen);
  node->cost = 0.0;
  node->parent = nullptr;
  return node;
}

// 查找最近的节点
Node* getNearestNode(Node* new_node, vector<Node*>& nodes) {
  Node* nearest_node = nodes[0];
  double min_distance = distance(new_node, nearest_node);
  for (auto& node : nodes) {
    double d = distance(node, new_node);
    if (d < min_distance) {
      nearest_node = node;
      min_distance = d;
    }
  }
  return nearest_node;
}

// 生成可行的子节点
vector<Node*> getFeasibleChildren(Node* parent, vector<Node*>& obstacles) {
  vector<Node*> children;
  double steer_angles[] = { -STEER_ANGLE_MAX, 0.0, STEER_ANGLE_MAX };
  double vels[] = { VEL_MAX / 2, VEL_MAX };
  for (auto& steer_angle : steer_angles) {
    for (auto& vel : vels) {
      double x = parent->x + vel * cos(parent->theta) * DT;
      double y = parent->y + vel * sin(parent->theta) * DT;
      double theta = parent->theta + vel / 2 * tan(steer_angle / 2 * DT);
      Node* child = new Node();
      child->x = x;
      child->y = y;
      child->theta = theta;
      child->cost = parent->cost + vel * DT;
      child->parent = parent;
      if (!isCollision(parent, child, obstacles)) {
        children.push_back(child);
      }
    }
  }
  return children;
}

// 查找附近的节点
vector<Node*> getNearNodes(Node* new_node, vector<Node*>& nodes, double radius) {
  vector<Node*> near_nodes;
  for (auto& node : nodes) {
    double d = distance(node, new_node);
    if (d < radius) {
      near_nodes.push_back(node);
    }
  }
  return near_nodes;
}

// 计算新节点的代价和路径是否更优
bool isBetterPath(Node* new_node, Node* nearest_node, vector<Node*>& obstacles) {
  double cost = nearest_node->cost + distance(new_node, nearest_node);
  if (cost >= new_node->cost) {
    return false;
  }
  if (isCollision(new_node, nearest_node, obstacles)) {
    return false;
  }
  Node* parent = nearest_node;
  while (parent != nullptr) {
    if (isCollision(new_node, parent, obstacles)) {
      return false;
    }
    parent = parent->parent;
  }
  new_node->cost = cost;
  new_node->parent = nearest_node;
  return true;
}

// 重连树
void rewire(Node* new_node, vector<Node*>& nodes, vector<Node*>& obstacles, double radius) {
  for (auto& node : nodes) {
    if (node == new_node || distance(node, new_node) > radius) {
      continue;
    }
    double cost = new_node->cost + distance(node, new_node);
    if (cost < node->cost && !isCollision(new_node, node, obstacles)) {
      Node* parent = node->parent;
      node->parent = new_node;
      node->cost = cost;
      while (parent != nullptr) {
        if (isCollision(node, parent, obstacles)) {
          break;
        }
        double parent_cost = parent->cost + distance(node, parent);
        if (parent_cost >= node->cost) {
          break;
        }
        node->cost = parent_cost;
        node->parent = parent;
        parent = parent->parent;
      }
    }
  }
}

// 搜索路径
vector<Node*> searchPath(Node* start_node, Node* goal_node, vector<Node*>& obstacles) {
  vector<Node *> nodes;
  nodes.push_back(start_node);
  while (true) {
// 生成随机点并查找最近的节点
    Node *new_node = getRandomNode();
    Node *nearest_node = getNearestNode(new_node, nodes);
    // 生成可行的子节点，并选择代价最小的子节点
    vector<Node *> children = getFeasibleChildren(nearest_node, obstacles);
    if (children.empty()) {
      continue;
    }
    Node *min_cost_child = children[0];
    for (auto &child: children) {
      if (child->cost < min_cost_child->cost) {
        min_cost_child = child;
      }
    }

    // 判断是否到达目标点，如果是则返回路径
    if (distance(min_cost_child, goal_node) < GOAL_TOL) {
      goal_node->parent = min_cost_child;
      goal_node->cost = min_cost_child->cost + distance(min_cost_child, goal_node);
      vector<Node *> path;
      Node *node = goal_node;
      while (node != nullptr) {
        path.insert(path.begin(), node);
        node = node->parent;
      }
      return path;
    }

    // 将代价最小的子节点添加到树中
    min_cost_child->parent = nearest_node;
    nodes.push_back(min_cost_child);

    // 重连树并更新节点代价
    rewire(min_cost_child, nodes, obstacles, 5.0);

    // 删除多余的节点
    for (auto it = nodes.begin()+1; it != nodes.end();) {
      if ((*it)->parent == nullptr) {
        delete (*it);
        it = nodes.erase(it);
      } else {
        ++it;
      }
    }
  }
}
#endif //RRTSTARTSMART_VEHICLE_RRTTEST_H
