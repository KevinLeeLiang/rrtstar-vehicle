#include <iostream>

#include <iostream>
#include <vector>
#include <cmath>
#include <random>

using namespace std;



//int main() {
//// 定义起点和终点
//  Node *start_node = new Node();
//  start_node->x = 5.0;
//  start_node->y = 5.0;
//  start_node->theta = PI / 2.0;
//  start_node->cost = 0.0;
//  start_node->parent = nullptr;
//  Node *goal_node = new Node();
//  goal_node->x = 45.0;
//  goal_node->y = 45.0;
//  goal_node->theta = PI / 2;
//  goal_node->cost = 0.0;
//  goal_node->parent = nullptr;
//
//// 定义障碍物
//  vector<Node *> obstacles;
////  Node *obstacle1 = new Node();
////  obstacle1->x = 20.0;
////  obstacle1->y = 25.0;
////  obstacle1->theta = 0.0;
////  obstacle1->cost = 0.0;
////  obstacle1->parent = nullptr;
////  obstacles.push_back(obstacle1);
////
////  Node *obstacle2 = new Node();
////  obstacle2->x = 30.0;
////  obstacle2->y = 35.0;
////  obstacle2->theta = 0.0;
////  obstacle2->cost = 0.0;
////  obstacle2->parent = nullptr;
////  obstacles.push_back(obstacle2);
////
////  Node *obstacle3 = new Node();
////  obstacle3->x = 40.0;
////  obstacle3->y = 25.0;
////  obstacle3->theta = 0.0;
////  obstacle3->cost = 0.0;
////  obstacle3->parent = nullptr;
////  obstacles.push_back(obstacle3);
//
//// 搜索路径
//  vector<Node *> path = searchPath(start_node, goal_node, obstacles);
//
//// 输出路径
//  for (auto &node: path) {
//    cout << "(" << node->x << ", " << node->y << ")" << endl;
//  }
//
//// 释放内存
//  delete start_node;
//  delete goal_node;
//  for (auto &obstacle: obstacles) {
//    delete obstacle;
//  }
//  for (auto &node: path) {
//    delete node;
//  }
//
//  return 0;
//}
#include"rrt_star.h"
// Define the main function
int main() {
  srand(time(NULL)); // Seed the random number generator
// Initialize the RRT*-smart algorithm object
  RRTStarSmart rrt;

// Set the start and goal points
  Point start(10.0, 10.0);
  Point goal(490.0, 490.0);

// Run the RRT*-smart algorithm
  vector<Point> path = rrt.run(start, goal);

// Print the path
  cout << "Path: ";
  for (int i = path.size() - 1; i >= 0; i--) {
    cout << path[i] << " ";
  }
  cout << endl;

  return 0;

}

