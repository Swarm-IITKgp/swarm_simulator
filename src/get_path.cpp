#include<vector>
#include "ros/ros.h"
#include<math.h>
#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"
#include<queue>
#include<stdio.h>
#include<stdlib.h>
#define gridx1 -12
#define gridx2 12
#define gridy1 -12
#define gridy2 12
#define maxx 100
#define maxy 100
#define thres 0.1
// struct for a point in path
typedef struct _pix
{
double x;
double y;
}pix;
class qi
{
public:
pix p;
double distance;//estimated total distance
};
struct comp
{
bool operator()(const qi& l, const qi& r)
{
return l.distance > r.distance;
}
};
bool operator==(const pix& lhs, const pix& rhs)
{
return ((lhs.x==rhs.x)&&(lhs.y==rhs.y));
}
bool operator!=(const pix& lhs, const pix& rhs)
{
return ((lhs.x!=rhs.x)||(lhs.y!=rhs.y));
}
/*struct op
{
bool operator()(const qi& lhs, const qi& rhs) const
{
return lhs.distance < rhs.distance;
}
};*/
/*class graph
{
public:
//int create();
static std::vector<pix> create(const swarm_simulator::obstacleList & obs,pix start, pix fin);
};*/
double dist(pix a, pix b)
{
return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}
int check(pix p,std::vector<swarm_simulator::obstacleData > obs )
{
pix temp;
if(p.x<0||p.x>maxx)
return 0;
if(p.y<0||p.y>maxy)
return 0;
for(std::vector<swarm_simulator::obstacleData>::iterator it=obs.begin();it!=obs.end();it++)
{
temp={(*it).x,(*it).y};
if(dist(temp,p)<(*it).radius)
return 0;
}
return 1;
}

// Function for creating sequence from obstacle list
std::list<pix > create(const std::vector<swarm_simulator::obstacleData > & obs,pix start, pix fin)
{
int i,j,k,l;
double temp,t2;
qi curr,tempq;
pix temppix;
std::priority_queue< qi ,std::vector<qi> ,comp> q;
std::vector<std::vector<pix> > parent(maxx+1);
std::vector<std::vector<double> > distances(maxx+1);
std::vector<pix> sequence;
//std::list<pix> sequence;
std::vector<std::vector<int> > visited(maxx+1);
for(i=0;i<=maxx;i++)
{
visited[i]=std::vector<int>(maxy+1);
distances[i]=std::vector<double>(maxy+1);
parent[i]=std::vector<pix>(maxy+1);
for(j=0;j<=maxy;j++)
{
visited[i][j]=0;
distances[i][j]=0;
}
}
curr.p=start;
curr.distance=dist(start,fin);
distances[start.x][start.y]=0;
q.push(curr);
while(!q.empty())
{
curr=q.top();
q.pop();
visited[curr.p.x][curr.p.y]=1;
//ROS_INFO("%lf %lf %lf",curr.p.x,curr.p.y,curr.distance);
//scanf("%d",&k);
if(curr.p==fin)
{
break;
}
for(i=curr.p.x-1;i<=curr.p.x+1;i++)
{
for(j=curr.p.y-1;j<=curr.p.y+1;j++)
{
temppix={i,j};
if(temppix==curr.p)
continue;
if(check(temppix,obs)==1)
{
t2=distances[curr.p.x][curr.p.y]+dist(temppix,curr.p);
temp=dist(temppix,fin)+t2;
if(visited[i][j]==1&&t2>=distances[i][j])
continue;
//ROS_INFO("%d %d %lf",i,j, temp);
//scanf("%d",&k);
parent[i][j]=curr.p;
tempq.p=temppix;
tempq.distance=temp;
distances[i][j]=t2;
q.push(tempq);
}
}
}
}
temppix=fin;
while(temppix!=start)
{
sequence.push_back(temppix);
temppix=parent[temppix.x][temppix.y];
}
return sequence;
}

// reducing no. of pionts in sequence
std::vector<pix>reduce(vector<pix> s)
{
int i=0,j;
double ori, orin;
pix temppix;
std::vector<pix> shortseq;
ori = (s[i+1].y-s[i].y)/(s[i+1].x-s[i].x);
for(i=1,j=0;i<=s.size();i++)
{
orin = (s[i].y-s[j].y)/(s[i].x-s[j].x);
if ((orin-ori)>=thres|| (ori-orin)>=thres)
{
temppix.x=s[i].x;
temppix.y=s[i].y;
shortseq.push_back(temppix);
}
}
return shortseq ;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "Bullshit");
//FILE *file;
//file=fopen("temp.txt","w");
std::list<pix> sequence;
pix start={1,1},fin={100,100};
// hardcoded obstacle list 
std::vector<swarm_simulator::obstacleData > x(3);

x[0].x=40;
x[0].y=40;
x[0].radius=5;
x[1].x=60;
x[1].y=13;
x[1].radius=7;
x[2].x=70;
x[2].y=90;
x[2].radius=5;
// sequence created from A*
sequence=create(x,start,fin);
// reduced sequence 
sequence=reduce(sequence);
for(std::list<pix>::iterator it=sequence.begin();it!=sequence.end();it++)
{
ROS_INFO("%lf\t%lf",(*it).x,(*it).y);
//printf("%lf %lf\n",(*it).x,(*it).y);
}
return 0;
}
