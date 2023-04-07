//
// Created by garen-lee on 2023/4/7.
//

#ifndef RRTSTARTSMART_VEHICLE_RRT_STAR_H
#define RRTSTARTSMART_VEHICLE_RRT_STAR_H
#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>

using namespace std;

// Define a class for representing 2D points
class Point
{
public:
  double x;
  double y;

  Point() { x = 0; y = 0; }
  Point(double _x, double _y) { x = _x; y = _y; }

  friend ostream& operator<<(ostream &strm, const Point &p) {
    strm << "(" << p.x << ", " << p.y << ")";
    return strm;
  }
};

// Define a class for representing tree nodes
class Node
{
public:
  Point point;
  int parent;

  Node() { parent = -1; }
  Node(Point _point, int _parent) { point = _point; parent = _parent; }
};

// Define a class for representing the RRT*-smart algorithm
class RRTStarSmart {
public:
  vector<Node> nodes;
  double stepSize;
  double goalRadius;
  double maxIterations;
  double minDistToObstacle;
  double mapWidth;
  double mapHeight;
  double obstacleDensity;
  double carWidth;
  double carLength;
  double maxSteeringAngle;
  double maxSpeed;
  double dt;

  RRTStarSmart() {
    // Set default values for algorithm parameters
    stepSize = 5.0;
    goalRadius = 10.0;
    maxIterations = 10000;
    minDistToObstacle = 10.0;
    mapWidth = 500.0;
    mapHeight = 500.0;
    obstacleDensity = 0.1;
    carWidth = 2.0;
    carLength = 4.0;
    maxSteeringAngle = 30.0 * M_PI / 180.0;
    maxSpeed = 10.0;
    dt = 0.1;
  }

  // Function to generate a random point within the map boundaries
  Point getRandomPoint() {
    double x = rand() / (double) RAND_MAX * mapWidth;
    double y = rand() / (double) RAND_MAX * mapHeight;
    return Point(x, y);
  }

  // Function to check if a point is within the obstacle
  bool isInObstacle(Point p) {
    double distToObstacle = 0.0;
    for (int i = 0; i < nodes.size(); i++) {
      distToObstacle = sqrt(pow(nodes[i].point.x - p.x, 2) + pow(nodes[i].point.y - p.y, 2));
      if (distToObstacle < minDistToObstacle) {
        return true;
      }
    }
    return false;
  }

  // Function to check if a point is close enough to the goal
  bool isCloseToGoal(Point p) {
    double distToGoal = sqrt(pow(p.x - mapWidth, 2) + pow(p.y - mapHeight, 2));
    if (distToGoal < goalRadius) {
      return true;
    }
    return false;
  }

  // Function to get the index of the nearest node to the given point
  int getNearestNodeIndex(Point p) {
    int nearestNodeIndex = -1;
    double minDist = INFINITY;
    double dist = 0.0;
    for (int i = 0; i < nodes.size(); i++) {
      dist = sqrt(pow(nodes[i].point.x - p.x, 2) + pow(nodes[i].point.y - p.y, 2));
      if (dist < minDist) {
        minDist = dist;
        nearestNodeIndex = i;
      }
    }
    return nearestNodeIndex;
  }

  // Function to get the angle between two points
  double getAngleBetweenPoints(Point a, Point b) {
    return atan2(b.y - a.y, b.x - a.x);
  }

  // Function to check if a new point is collision-free
  // TODO: re code new Collision free
  bool isCollisionFree(Point p, double steeringAngle, double speed) {
    // Calculate the car's turning radius and turning circle center
//    double turningRadius = speed / tan(steeringAngle);
//    Point turningCircleCenter(
//        p.x - turningRadius * sin(getAngleBetweenPoints(Point(p.x, p.y), nodes[nodes[p.parent].parent].point)),
//        p.y + turningRadius * cos(getAngleBetweenPoints(Point(p.x, p.y), nodes[nodes[p.parent].parent].point)));
//
//    // Check if any part of the car
//    // is inside an obstacle
//    Point carFront(turningCircleCenter.x + (turningRadius + carLength / 2.0) *
//                                           cos(getAngleBetweenPoints(Point(p.x, p.y),
//                                                                     nodes[nodes[p.parent].parent].point) + M_PI / 2.0 -
//                                               steeringAngle), turningCircleCenter.y +
//                                                               (turningRadius + carLength / 2.0) *
//                                                               sin(getAngleBetweenPoints(Point(p.x, p.y),
//                                                                                         nodes[nodes[p.parent].parent].point) +
//                                                                   M_PI / 2.0 - steeringAngle));
//    Point carRear(turningCircleCenter.x + (turningRadius - carLength / 2.0) * cos(getAngleBetweenPoints(Point(p.x, p.y),
//                                                                                                        nodes[nodes[p.parent].parent].point) +
//                                                                                  M_PI / 2.0 - steeringAngle),
//                  turningCircleCenter.y + (turningRadius - carLength / 2.0) * sin(getAngleBetweenPoints(Point(p.x, p.y),
//                                                                                                        nodes[nodes[p.parent].parent].point) +
//                                                                                  M_PI / 2.0 - steeringAngle));
//    Point carLeftFront(carFront.x - carWidth / 2.0 *
//                                    sin(getAngleBetweenPoints(Point(p.x, p.y), nodes[nodes[p.parent].parent].point) +
//                                        M_PI / 2.0 - steeringAngle), carFront.y + carWidth / 2.0 *
//                                                                                  cos(getAngleBetweenPoints(
//                                                                                      Point(p.x, p.y),
//                                                                                      nodes[nodes[p.parent].parent].point) +
//                                                                                      M_PI / 2.0 - steeringAngle));
//    Point carRightFront(carFront.x + carWidth / 2.0 *
//                                     sin(getAngleBetweenPoints(Point(p.x, p.y), nodes[nodes[p.parent].parent].point) +
//                                         M_PI / 2.0 - steeringAngle), carFront.y - carWidth / 2.0 *
//                                                                                   cos(getAngleBetweenPoints(
//                                                                                       Point(p.x, p.y),
//                                                                                       nodes[nodes[p.parent].parent].point) +
//                                                                                       M_PI / 2.0 - steeringAngle));
//    Point carLeftRear(carRear.x - carWidth / 2.0 *
//                                  sin(getAngleBetweenPoints(Point(p.x, p.y), nodes[nodes[p.parent].parent].point) +
//                                      M_PI / 2.0 - steeringAngle), carRear.y + carWidth / 2.0 *
//                                                                               cos(getAngleBetweenPoints(
//                                                                                   Point(p.x, p.y),
//                                                                                   nodes[nodes[p.parent].parent].point) +
//                                                                                   M_PI / 2.0 - steeringAngle));
//    Point carRightRear(carRear.x + carWidth / 2.0 *
//                                   sin(getAngleBetweenPoints(Point(p.x, p.y), nodes[nodes[p.parent].parent].point) +
//                                       M_PI / 2.0 - steeringAngle), carRear.y - carWidth / 2.0 *
//                                                                                cos(getAngleBetweenPoints(
//                                                                                    Point(p.x, p.y),
//                                                                                    nodes[nodes[p.parent].parent].point) +
//                                                                                    M_PI / 2.0 - steeringAngle));
//
//    if (isInObstacle(carFront) || isInObstacle(carRear) || isInObstacle(carLeftFront) || isInObstacle(carRightFront) ||
//        isInObstacle(carLeftRear) || isInObstacle(carRightRear)) {
//      return false;
//    }

    return true;
  }

// Function to add a new node to the tree
  int addNewNode(Point p, int nearestNodeIndex) {
    Node newNode = Node(p, nearestNodeIndex);
    nodes.push_back(newNode);
    return nodes.size() - 1;
  }

// Function to rewire the tree based on the new node
// TODO: re code
  void rewireTree(int newNodeIndex) {
    for (int i = 0; i < nodes.size(); i++) {
      if (i == newNodeIndex || nodes[i].parent == -1) {
        continue;
      }

      double distFromNodeToNewNode = sqrt(pow(nodes[i].point.x - nodes[newNodeIndex].point.x, 2) +
                                          pow(nodes[i].point.y - nodes[newNodeIndex].point.y, 2));
      double newCost =
          nodes[newNodeIndex].point.x - nodes[nodes[newNodeIndex].parent].point.x + nodes[newNodeIndex].point.y -
          nodes[nodes[newNodeIndex].parent].point.y;
      double oldCost =
          nodes[i].point.x - nodes[nodes[i].parent].point.x + nodes[i].point.y - nodes[nodes[i].parent].point.y;

      if (newCost < oldCost && distFromNodeToNewNode < stepSize) {
        if (isCollisionFree(nodes[i].point,
                            getAngleBetweenPoints(nodes[newNodeIndex].point, nodes[nodes[newNodeIndex].parent].point),
                            maxSpeed)) {
          nodes[i].parent = newNodeIndex;
        }
      }
    }
  }

  // Function to run the RRT*-smart algorithm
  vector<Point> run(Point start, Point goal) {
    // Initialize tree with start node
    Node startNode = Node(start, -1);
    nodes.push_back(startNode);

    int iterCount = 0;
    Point randomPoint, nearestPoint, newPointTowardsGoal;
    int nearestNodeIndex, newNodeIndex;

    while (iterCount < maxIterations) {
      // Generate a random point in the map
      randomPoint = getRandomPoint();

      // Find the nearest node to the random point
      nearestNodeIndex = getNearestNodeIndex(randomPoint);
      nearestPoint = nodes[nearestNodeIndex].point;

      // Get a new point towards the goal from the nearest node
      double angleBetweenPoints = getAngleBetweenPoints(nearestPoint, randomPoint);
      double steeringAngle =
          maxSteeringAngle * atan(2.0 * carLength * sin(angleBetweenPoints) / (maxSpeed * dt)) / M_PI;
      double speed = maxSpeed * cos(steeringAngle);
      Point newPoint = Point(nearestPoint.x + speed * dt * cos(angleBetweenPoints),
                             nearestPoint.y + speed * dt * sin(angleBetweenPoints));
      newPointTowardsGoal = Point(newPoint.x + maxSpeed * dt * cos(getAngleBetweenPoints(newPoint, goal)),
                                  newPoint.y + maxSpeed * dt * sin(getAngleBetweenPoints(newPoint, goal)));

      // If the new point is collision-free, add it as a new node and rewire the tree
      if (isCollisionFree(newPoint, steeringAngle, speed) &&
          isCollisionFree(newPointTowardsGoal, getAngleBetweenPoints(newPoint, goal), maxSpeed)) {

        // If the random point is close enough to the goal, set it as the goal and return the path
        if (isCloseToGoal(newPoint)) {
          nodes.push_back(Node(goal, -1));
          int goalNodeIndex = nodes.size() - 1;
          nodes[goalNodeIndex].parent = nearestNodeIndex;
          vector<Point> path;

          std::cout << "arrived goalNodeIndex:" << goalNodeIndex << std::endl;
          std::cout << "node size:" << nodes.size() << std::endl;
          int currentNodeIndex = goalNodeIndex;
          while (currentNodeIndex != -1) {
            path.push_back(nodes[currentNodeIndex].point);
            currentNodeIndex = nodes[currentNodeIndex].parent;
          }
          return path;
        }

        newNodeIndex = addNewNode(newPoint, nearestNodeIndex);
        rewireTree(newNodeIndex);
      }

      iterCount++;
    }

    // Return an empty path if no path was found
    vector<Point> path;
    return path;
  }
};

#endif //RRTSTARTSMART_VEHICLE_RRT_STAR_H
