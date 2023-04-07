//
// Created by garen-lee on 2023/4/4.
//

#include "RRTStarSmartPlanner.h"
RRTStarSmartPlanner::RRTStarSmartPlanner(double goal_max_distance_, double neighbor_radius_, double speed_max_, double L_, double dt_) :
    goal_max_distance(goal_max_distance_), neighbor_radius(neighbor_radius_), speed_max(speed_max_), L(L_), dt(dt_) {}

// 添加初始节点
void RRTStarSmartPlanner::addStartNode(const State& start_state) {
  Node* start_node = new Node(new State(start_state));
  tree.push_back(start_node);
}

// 添加目标节点
void RRTStarSmartPlanner::addGoalNode(const State& goal_state) {
  Node* goal_node = new Node(new State(goal_state));
  tree.push_back(goal_node);
}

// 获取随机节点
Node* RRTStarSmartPlanner::getRandomNode() {
  random_device rd;
  mt19937 gen(rd());
  uniform_real_distribution<> x_dist(-10.0, 10.0);
  uniform_real_distribution<> y_dist(-10.0, 10.0);
  uniform_real_distribution<> theta_dist(-M_PI, M_PI);
  uniform_real_distribution<> v_dist(0.0, speed_max);

  double x = x_dist(gen);
  double y = y_dist(gen);
  double theta = theta_dist(gen);
  double v = v_dist(gen);
  State* state = new State(x, y, theta, v);
  Node* node = new Node(state);
  return node;
}

// 获取距离point最近的节点
Node* RRTStarSmartPlanner::getNearestNode(vector<Node*>& nodes, const State& point) {
  Node* nearest_node = nodes[0];
  double min_distance = getDistance(*(nearest_node->state), point);
  for (auto node : nodes) {
    double distance = getDistance(*(node->state), point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }
  return nearest_node;
}

// 获取新节点
Node* RRTStarSmartPlanner::getNewNode(Node* nearest_node, const State& rand_state) {
  double steering_angle = atan2(rand_state.y - nearest_node->state->y, rand_state.x - nearest_node->state->x) - nearest_node->state->theta;
  if (steering_angle > M_PI) {
    steering_angle -= 2 * M_PI;
  } else if (steering_angle < -M_PI) {
    steering_angle += 2 * M_PI;
  }
  double delta_t = dt;
  double v = getVelocity(nearest_node->state, rand_state);
  double x = nearest_node->state->x + v * cos(nearest_node->state->theta) * delta_t;
  double y = nearest_node->state->y + v * sin(nearest_node->state->theta) * delta_t;
  double theta = nearest_node->state->theta + v / L * tan(steering_angle) * delta_t;

  if (theta > M_PI) {
    theta -= 2 * M_PI;
  } else if (theta < -M_PI) {
    theta += 2 * M_PI;
  }

  State* state = new State(x, y, theta, v);
  Node* node = new Node(state);
  return node;
}

vector<Node*> RRTStarSmartPlanner::findNearNodes(vector<Node*>& tree, const State& point) {
  vector<Node*> nearNodes;

  // 计算最大扩展距离
  double searchRadius = min(maxSearchRadius, maxStepSize * pow(log(tree.size() + 1) / tree.size(), 1.0 / (double)numDims));

  // 在搜索树中查找所有节点
  for (int i = 0; i < tree.size(); i++) {
    Node* node = tree[i];

    // 如果节点到目标点的距离小于阈值，则���其添加到nearNodes列表中
    if (distance(node->getState(), point) <= searchRadius) {
      nearNodes.push_back(node);
    }
  }

  return nearNodes;
}
bool RRTStarSmartPlanner::isCollisionFree(Node* node1, Node* node2) {
  // 获取节点之间的路径
  vector<State> path = getPath(node1, node2);

  // 检查路径上每个点是否与障碍物相交
  for (int i = 0; i < path.size(); i++) {
    if (!isStateValid(path[i])) {
      return false;
    }
  }

  // 如果路径上所有点都合法，则返回true
  return true;
}

