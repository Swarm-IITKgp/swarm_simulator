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

Pose current_pos; // global updated by callback function
std::vector<swarm_simulator::obstacleData> obstacles; // global updated by callback function
bool new_obstacles = false;
bool pos_updated = false;

const int id = 0;

// the arena on the simulator stretches from (-X_MAX/2, -Y_MAX/2) to (X_MAX/2, Y_MAX/2) 
const int X_MAX = 20;
const int Y_MAX = 20;
const int SCALE = 1; // Each cell of map[][] covers 1/SCALE on simulator
// hence higher the scale, more refined the path generated.
const double TOL = 1 / double(SCALE);

int map[X_MAX * SCALE][Y_MAX * SCALE];
//map[i][j] = 0 if passable
//          = 1 if in obstacle
//          = 2 if in path
//          = 3 if in frontier considered by A*




void odomCB(const Odom& msg) {

  std::cout << "OdomCB called" << std::endl;
  current_pos = msg.pose.pose;

  pos_updated = true;

  std::cout << "CB_current_pos = " << current_pos.position.x << ", " << current_pos.position.y << std::endl;
}



void obstacleCB(const swarm_simulator::obstacleList msg) {
  std::cout << "msg.obstacles.size() = " << msg.obstacles.size() << std::endl;

  new_obstacles = true;


  obstacles.clear();
  for(int i = 0; i<msg.obstacles.size(); ++i){
    if((msg.obstacles[i].radius > 1.4 && msg.obstacles[i].radius < 1.6) && msg.obstacles[i].shape==id) {
    }
    else {
      obstacles.push_back(msg.obstacles[i]);
    }
  }

  std::cout << "obstacles.size() = " << obstacles.size() << std::endl;
}




////////////////////
// Motion Control //
////////////////////

void calculate_u_omega(Pose current_, Pose destination_, Twist& cmd_vel);

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
void fillMap(std::vector<swarm_simulator::obstacleData> obstacles) {

  for(int i = 0; i < X_MAX; ++i) {
    for(int j = 0; j < Y_MAX; ++j) {
      map[i][j] = 0;
    }
  }

  for(auto it = obstacles.begin(); it != obstacles.end(); ++it) {
    auto obstacle = *it;

    double x = (obstacle.x + X_MAX / 2) * SCALE;
    double y = (obstacle.y + Y_MAX / 2) * SCALE;
    double radius = obstacle.radius * SCALE;
    for(int i = x - radius; i < x + radius; ++i) {
      for(int j = y - radius; j < y + radius; ++j) {
        if (i >= 0 && i < X_MAX * SCALE && j >= 0 && j < Y_MAX * SCALE) {
          if ((i - x)*(i - x) + (j - y)*(j - y) <= radius*radius) {
            map[i][j] = 1;
          }
        }
      }
    } 
  }
}

void printMap() {
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

typedef std::pair<int, int> Point;

inline double graph_cost(Point a, Point b) {
  return sqrt((a.first - b.first)*(a.first - b.first) + (a.second - b.second)*(a.second - b.second));
}

inline double heuristic(Point a, Point b) {
  return graph_cost(a, b);
}

// returns whether a point is traversable
inline bool passable(Point a) {
  if (a.first >= 0 && a.first < X_MAX * SCALE && a.second >= 0 && a.second < Y_MAX * SCALE) {
    if (map[a.first][a.second] == 0) {
      return true;
    }
    else return false;
  }
  else return false;
}

// Returns a vector of the valid neighbors of current
std::vector<Point> neighborList
(Point current) {
  std::vector<Point> neighbors;
  
  Point neighbor;

  neighbor = Point(current.first - 1, current.second - 1);
  if(passable(neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first - 1, current.second);
  if(passable(neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first - 1, current.second + 1);
  if(passable(neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first, current.second - 1);
  if(passable(neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first, current.second + 1);
  if(passable(neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first + 1, current.second - 1);
  if(passable(neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first + 1, current.second);
  if(passable(neighbor)) neighbors.push_back(neighbor);

  neighbor = Point(current.first + 1, current.second + 1);
  if(passable(neighbor)) neighbors.push_back(neighbor);

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

std::vector<Point> AStar(Point source, Point destination) {
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

    std::vector<Point> neighbors = neighborList(current);

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


int main(int argc, char **argv)
{


  ros::init(argc, argv, "MotionController");

  ros::NodeHandle n;

  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 50);
  ros::Subscriber odom_data = n.subscribe("/swarmbot0/odom", 50,odomCB);
  ros::Subscriber obstacle_data = n.subscribe("/obstacleList", 50, obstacleCB);
  ros::Rate loop_rate(50);


  Pose destination;
  Twist cmd_vel;

  destination.position.y = destination.position.x = -10; //start position
  destination.orientation.z = sin(M_PI/2);
  destination.orientation.w = cos(M_PI/2);
  destination.orientation.x = destination.orientation.y = destination.position.z = 0;  

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = cmd_vel.angular.y = cmd_vel.angular.x = cmd_vel.linear.y = cmd_vel.linear.z = 0;

  std::vector<Point> path;


  while(ros::ok())
  {
    ros::spinOnce();

    while(obstacles.empty() || !pos_updated) {
      std::cout << "Waiting for obstacle list" <<  std::endl;
      ros::spinOnce();
      loop_rate.sleep();
    }

    //std::cout << "Out of loop" << std::endl;

    if(new_obstacles == true) { //obstacles updated, find new path
      fillMap(obstacles);


      //printMap();




      long current_x = std::lround((current_pos.position.x + X_MAX / 2) * SCALE);
      long current_y = std::lround((current_pos.position.y + Y_MAX / 2) * SCALE);

      std::cout << "simu_current_pos = " << current_pos.position.x << ", " << current_pos.position.y << std::endl;
      std::cout << "map_current_pos = " << current_x << ", " << current_y << std::endl;

      if(current_x < 0) current_x = 0;
      if(current_y < 0) current_y = 0;
      if(current_x >= X_MAX * SCALE) current_x = X_MAX * SCALE - 1;
      if(current_y >= Y_MAX * SCALE) current_y = Y_MAX * SCALE - 1;

      path = AStar(Point(current_x, current_y), Point(X_MAX * SCALE - 1, Y_MAX * SCALE - 1));

      printMap();

      new_obstacles = false;

    }

    

    // if (!found_path) {
    //   path = AStar(Point(0, 0), Point(38, 38));
    //   printMap();
    //   found_path = true;
    // }

    // std::cout << "destination = " << destination.position.x << ", " << destination.position.y << std::endl; 
    // std::cout << "current_pos = " << current_pos.position.x << ", " << current_pos.position.y << std::endl;
    // std::cout << "path.size() = " << path.size() << std::endl;

    if (!path.empty()) {
      if(current_pos.position.x > destination.position.x - TOL &&
       current_pos.position.x < destination.position.x + TOL &&
       current_pos.position.y > destination.position.y - TOL &&
       current_pos.position.y < destination.position.y + TOL) {


        path.pop_back();
      Point target = path.back();
      destination.position.x = (target.first / SCALE) - (X_MAX / 2);
      destination.position.y = (target.second / SCALE) - (Y_MAX / 2);

      std::cout << "simu_next_pos = " << destination.position.x << ", " << destination.position.y << std::endl;
      std::cout << "map_next_pos = " << target.first << ", " << target.second << std::endl;

      // if(!path.empty()) {
      //   Point next = path.back();
      //   double angle = normalizeAngle(atan2(next.second - target.second, next.first - target.first));
      //   destination.orientation.z = sin(angle/2);
      //   destination.orientation.w = cos(angle/2);
      // }
    }
  }

  calculate_u_omega(current_pos, destination, cmd_vel);
  vel_pub_0.publish(cmd_vel);

  loop_rate.sleep();
}

return 0;
}
