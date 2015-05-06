#ifndef _PATH_PLANNING_H
#define _PATH_PLANNING_H

#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"


#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <utility>


typedef typename geometry_msgs::Pose Pose;
typedef typename nav_msgs::Odometry Odom;
typedef typename geometry_msgs::Twist Twist;
typedef std::pair<int, int> Point;


////////////////////
// Motion Control //
////////////////////

//void calculate_u_omega(Pose current_, Pose destination_, Twist& cmd_vel);

double normalizeAngle(double angle) {
  angle = fmod(angle, 2*M_PI); // (0, 2PI)
  if(angle > M_PI) angle -= 2*M_PI; // (-PI, PI)

  return angle;
}

//In 2D, the quaternions are <0, 0, sin(theta/2), cos(theta/2)> = <x, y, z, w>
double theta(Pose pos) { //returns theta of pos wrt x-axis
  return normalizeAngle(2 * atan2(pos.orientation.z, pos.orientation.w));
    // atan2(y, x) is similar to atan(y/x) but returns the angle in the correct quadrant
    // ie, (-PI, PI) rather than (-PI/2, PI/2)
}

double theta(Pose a, Pose b) {
  return normalizeAngle(atan2(a.position.y - b.position.y, a.position.x - b.position.x));
}

double dist(Pose a, Pose b) {
  double temp = 0;
  temp += (a.position.x - b.position.x)*(a.position.x - b.position.x);
  temp += (a.position.y - b.position.y)*(a.position.y - b.position.y);
  return sqrt(temp);
}

//Inputs passed by value so cannot be changed by odomCB 
void calculate_u_omega(Pose current_, Pose destination_, Twist& cmd_vel) {
  double phi = normalizeAngle(theta(current_) - theta(destination_)); // orntn of crnt wrt orntn of dest
  double theta_ = normalizeAngle(theta(destination_, current_) - theta(destination_)); // angle of dest wrt crnt
  double alpha = normalizeAngle(theta_ - phi); // refer polar.pdf
  double e = dist(destination_, current_);

  const double gamma = 2;
  const double k = 1;
  const double h = 1;

  double u = e * gamma * cos(alpha);
  double w = (k * alpha) + (gamma * cos(alpha) * sin(alpha) / alpha) * (alpha + h * theta_);
  
  cmd_vel.linear.x = u;
  cmd_vel.angular.z = w;
  cmd_vel.linear.y = cmd_vel.linear.z = cmd_vel.angular.x = cmd_vel.angular.y = 0;
}
// Refer: Closed Loop Steering of Unicycle-like Vehicles via Lyapunov Techniques
// by M.Aicardi, G.Casalino, A. Bicchi, A.Balestrino

/////////////////////////////
// Path Planning (with A*) //
/////////////////////////////

// fills the 2D array with the obstacles
void fillMap(int **map, int X_MAX, int Y_MAX, int SCALE, std::vector<swarm_simulator::obstacleData> obstacles) {

  //std::cout << "fillMap called" << std::endl;

  for(int i = 0; i < X_MAX * SCALE; ++i) {
    for(int j = 0; j < Y_MAX * SCALE; ++j) {
      map[i][j] = 0;
    }
  }

  //std::cout << "fillMap1" << std::endl;


  for(auto it = obstacles.begin(); it != obstacles.end(); ++it) {
    auto obstacle = *it;

    double x = (obstacle.x + X_MAX / 2) * SCALE;
    double y = (obstacle.y + Y_MAX / 2) * SCALE;
    double radius = obstacle.radius * SCALE;
    for(int i = lround(x - radius); i <= lround(x + radius); ++i) {
      for(int j = lround(y - radius); j <= lround(y + radius); ++j) {
        if (i >= 0 && i < X_MAX * SCALE && j >= 0 && j < Y_MAX * SCALE) {
          if ((i - x)*(i - x) + (j - y)*(j - y) <= radius * radius) {
            map[i][j] = 1;
          }
        }
      }
    } 
  }
}

void printMap(int **map, int X_MAX, int Y_MAX, int SCALE) {
 // std::cout << "printMap called" << std::endl;

  for(int i = Y_MAX * SCALE - 1; i >= 0;  --i) {
    for(int j = 0; j < X_MAX * SCALE; ++j) {
      if(map[j][i] == 0) std::cout << "."; // free cell
      else if(map[j][i] == 1) std::cout << "#"; //obstacle
      else if(map[j][i] == 2) std::cout << "@"; //path
      else if(map[j][i] == 3) std::cout << "="; //frontier
      else std::cout << "?"; //garbage
    }
    std::cout << std::endl;
  }
}

inline double graph_cost(Point a, Point b) {
  return sqrt((a.first - b.first)*(a.first - b.first) + (a.second - b.second)*(a.second - b.second));
}

inline double heuristic(Point a, Point b) {
  return graph_cost(a, b);
}

// returns whether a point is traversable
inline bool passable(int **map, int X_MAX, int Y_MAX, int SCALE, Point a) {
  if (a.first >= 0 && a.first < X_MAX * SCALE && a.second >= 0 && a.second < Y_MAX * SCALE) {
    if (map[a.first][a.second] == 0) {
      return true;
    }
    else return false;
  }
  else return false;
}

// Returns a vector of the valid neighbors of current
std::vector<Point> neighborList(int **map, int X_MAX, int Y_MAX, int SCALE, Point current) {
  std::vector<Point> neighbors;
  
  Point neighbor;

  neighbor = Point(current.first - 1, current.second - 1);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first - 1, current.second);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first - 1, current.second + 1);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first, current.second - 1);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first, current.second + 1);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first + 1, current.second - 1);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first + 1, current.second);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first + 1, current.second + 1);
  if(passable(map, X_MAX, Y_MAX, SCALE, neighbor)) neighbors.push_back(neighbor);

  return neighbors;
}

class Comparator{ //functor for ASTAR priority queue
public:
  static Point destination;
  static std::map<Point, double> cost;
  // tried using statics instead of globals but didnt work

  inline bool operator()(Point a, Point b) {
    return (heuristic(a, destination) + cost[a]) > (heuristic(b, destination) + cost[b]);
  }
};

Point Comparator::destination = Point();
std::map<Point, double> Comparator::cost = std::map<Point, double>();

std::vector<Point> AStar(int **map, int X_MAX, int Y_MAX, int SCALE, Point source, Point destination) {

  //std::cout << "AStar called" << std::endl;

  std::map<Point, Point> parent;

  std::map<Point, double>& cost = Comparator::cost;

  cost.clear();
  Comparator::destination = destination;

  parent[source] = source;
  cost[source] = 0;
  std::priority_queue<Point, std::vector<Point>, Comparator> frontier;

  frontier.push(source);

  while(!frontier.empty()) {
    Point current = frontier.top();
    frontier.pop();

    if (current == destination) {
      break;
    }

    std::vector<Point> neighbors = neighborList(map, X_MAX, Y_MAX, SCALE, current);

    for(Point next: neighbors) {
      double new_cost = cost[current] + graph_cost(current, next);
      if(!cost.count(next) || new_cost < cost[next]) {
      //next has never been discovered or a shorter path is found
        cost[next] = new_cost;
        parent[next] = current;
        // double priority = new_cost + heuristic(next, destination);
        map[next.first][next.second] = 3;
        frontier.push(next);
      }
    }
  }

  //Reconstruct path from parent
  std::vector<Point> path;
  Point current = destination;
  path.push_back(current);  
  map[current.first][current.second] = 2;
  while(current != source) {
    current = parent[current];
    path.push_back(current);
    map[current.first][current.second] = 2;
  }

  return path; // end to start
}


#endif
