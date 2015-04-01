#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <queue> 

using namespace std;

std::vector<swarm_simulator::obstacleData> obstacles; // global updated by callback function

// the arena on the simulator stretches from (-X_MAX/2, -Y_MAX/2) to (X_MAX/2, Y_MAX/2) 
const int X_MAX = 20;
const int Y_MAX = 20;
const int SCALE = 2; // Each cell of map[][] covers 1/SCALE on simulator
// hence higher the scale, more refined the path generated.
const double TOL = 1 / double(SCALE);

int mapArray[X_MAX * SCALE][Y_MAX * SCALE];

class CNode
{
public:

    CNode() : xPos(0), yPos(0), travelCost(0) {}
    CNode(int x, int y) : xPos(x), yPos(y), travelCost(0) {}
    CNode(int x, int y, int cost) : xPos(x), yPos(y), travelCost(cost) {}

    inline CNode& operator=(const CNode& target)
    {
        if (*this != target)
        {
            xPos = target.xPos;
            yPos = target.yPos;
            travelCost = target.travelCost;
        }

        return *this;
    }

    inline bool operator==(const CNode& target) const
    {
        return xPos == target.xPos && yPos == target.yPos;
    }

    inline bool operator!=(const CNode& target) const
    {
        return !(*this == target);
    }

    inline bool operator<(const CNode& target) const
    {
        return target.travelCost < travelCost;
    }

    int xPos, yPos, travelCost;
};

class CPath
{
public:

    typedef vector<CNode> nodeList;

    nodeList Find(const CNode& startNode, const CNode& endNode, int mapArray[][Y_MAX * SCALE])
    {
        nodeList finalPath, openList, closedList;

        finalPath.push_back(startNode);
        openList.push_back(startNode);
        closedList.push_back(startNode);

        while (!openList.empty())
        {
            // Check each node in the open list
            for (int i = 0; i < openList.size(); ++i)
            {
                if (openList[i].xPos == endNode.xPos && openList[i].yPos == endNode.yPos)
                    return finalPath;

                priority_queue<CNode> nodeQueue;

                // Get surrounding nodes
                for (int x = -1; x <= 1; ++x)
                {
                    for (int y = -1; y <= 1; ++y)
                    {
                        const int current_x = openList[i].xPos + x;
                        const int current_y = openList[i].yPos + y;

                        bool alreadyCheckedNode = false;
                        for (int j = 0; j < closedList.size(); ++j)
                        {
                            if (current_x == closedList[j].xPos && current_y == closedList[j].yPos)
                            {
                                alreadyCheckedNode = true;
                                break;
                            }
                        }

                        if (alreadyCheckedNode)
                            continue;

                        // Ignore current coordinate and don't go out of array scope
                        if (current_x < 0 || current_x > X_MAX * SCALE || current_y < 0 ||current_y > Y_MAX * SCALE || (openList[i].xPos == current_x && openList[i].yPos == current_y))
                            continue;


                        if(x > 1 && y > 1)
                            openList.pop_back();
                                                

                        // Ignore walls
                        if (mapArray[current_x][current_y] == 1)
                          {  std::cout<<"in loop" ; continue; }

                        const int xNodeDifference = abs(current_x - (openList[i].xPos));
                        const int yNodeDifference = abs(current_y - (openList[i].yPos));            

                        // Diagonal?
                        const int direction = xNodeDifference == 1 && yNodeDifference == 1 ? 14 : 10;

                        const int xDistance = abs(current_x - endNode.xPos);
                        const int yDistance = abs(current_y - endNode.yPos);
                        
                        int heuristic = 10 * (xDistance + yDistance);

                        nodeQueue.push(CNode(current_x, current_y, heuristic));
                    }
                }

                if (!nodeQueue.empty())
                {
                    // Add the nearest node
                    openList.push_back(nodeQueue.top());
                    finalPath.push_back(nodeQueue.top());

                    // Put into closed list
                    while (!nodeQueue.empty())
                    {
                        closedList.push_back(nodeQueue.top());
                        nodeQueue.pop();
                    }
                }
            }
        }

        return finalPath;
    }
};

void obstacleCB(const swarm_simulator::obstacleList msg) {
  if(obstacles.empty())
    obstacles = msg.obstacles;
}

void populateMap(std::vector<swarm_simulator::obstacleData> obstacles) {
  for(int i = 0; i < X_MAX; ++i) {
    for(int j = 0; j < Y_MAX; ++j) {
      mapArray[i][j] = 0;
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
            mapArray[i][j] = 1;
          } 
        }
      }
    } 
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MotionController");

    ros::NodeHandle n;

    ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 50);
    ros::Subscriber obstacle_data = n.subscribe("/obstacleList", 50, obstacleCB);
    ros::Rate loop_rate(50);
    CNode start, end;
    CPath pathFinder;
    while(ros::ok()) {
        ros::spinOnce();

        while(obstacles.empty()) {
            std::cout << "Waiting for obstacle list" <<  std::endl;
            ros::spinOnce();
            loop_rate.sleep();
        }

        populateMap(obstacles);

    mapArray[0][0] = 5;
    mapArray[38][38] = 6;
    for (int width = 0; width < X_MAX * SCALE; ++width) {
        for (int height = 0; height < Y_MAX * SCALE; ++height) {
            if (mapArray[width][height] == 5) {
                start.xPos = width;
                start.yPos = height;
            } else if (mapArray[width][height] == 6) {
                end.xPos = width;
                end.yPos = height;
            }
        }
    }

    
    CPath::nodeList n = pathFinder.Find(start, end, mapArray);

    for (int i = 0; i < n.size(); ++i) {
        if (mapArray[n[i].xPos][n[i].yPos] != 5 && mapArray[n[i].xPos][n[i].yPos] != 6)
            mapArray[n[i].xPos][n[i].yPos] = 2;
    }

            for(int i = Y_MAX * SCALE - 1; i >= 0;  --i) {
            for(int j = 0; j < X_MAX * SCALE; ++j) {
                if(mapArray[j][i] == 0) std::cout << "."; // free cell
                else if(mapArray[j][i] == 1) std::cout << "#"; //obstacle
                else if(mapArray[j][i] == 2) std::cout << "*"; //path
                else if(mapArray[j][i] == 5) std::cout << "A" ;
                else if(mapArray[j][i] == 6) std::cout << "B" ;
            else std::cout << "?"; //garbage
    }
    std::cout << std::endl;
    }

        cin.get();
    }
    // for (int width = 0; width < 20; ++width) {
    //     for (int height = 0; height < 20; ++height) {
    //         if (mapArray[width][height] == 'A') {
    //             start.xPos = width;
    //             start.yPos = height;
    //         } else if (mapArray[width][height] == 'B') {
    //             end.xPos = width;
    //             end.yPos = height;
    //         }
    //     }
    // }

    // CPath pathFinder;
    // CPath::nodeList n = pathFinder.Find(start, end, mapArray);

    // for (int i = 0; i < n.size(); ++i)
    //     if (mapArray[n[i].xPos][n[i].yPos] != 'A' && mapArray[n[i].xPos][n[i].yPos] != 'B')
    //         mapArray[n[i].xPos][n[i].yPos] = '*';

    // for (int height = 0; height < 20; ++height) {
    //     for (int width = 0; width < 20; ++width) {
    //         if (width % 20 == 0)
    //             cout << endl;
    //         cout << (char)mapArray[height][width] << " ";
    //     }
    // }

    cin.get();

    return 0;
}