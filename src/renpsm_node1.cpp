/*
    This file is part of RENPSM.
    
    https://github.com/RGNC/renpsm

    RENPSM is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    RENPSM is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with RENPSM.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>	
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <renpsm/tfListener.hpp>
#include <renpsm/rosmap.hpp>
#include <renpsm/renpsm.hpp>
#include <string>


namespace renpsm
{


class RosObstacles : public Obstacles
{
public:
	RosObstacles() {}
	virtual ~RosObstacles() {}
	virtual double getDistanceToNearestObstacle(int i, int j)
	{
		utils::RosMap::Pixel p;
		p.x = i;
		p.y = j;
		utils::Vector2d x;
		utils::MAP.pixelToMap(p,x);
		double d = utils::MAP.getNearestObstacle(x).distance;
		return d / utils::MAP.getInfo().resolution;
	}
} ROS_OBSTACLES;



class Node
{
public:
	Node(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Node() {}



private:

	void goalReceived(const geometry_msgs::PoseStamped::ConstPtr& goal); 
	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);

	void drawRRT();
	void drawPath();
	void drawRRT(Membrane *root, visualization_msgs::Marker& marker);
	void initMarker(visualization_msgs::Marker& marker);

	

	int x_init;
	int y_init;
	int x_end;
	int y_end;
	int x_max;
	int y_max;
	double d_min;
	double delta_t;
	double robot_radius;

	BidirectionalRRT algorithm;

	ros::Publisher rrt_pub;
	ros::Publisher path_pub;
	
};


void Node::goalReceived(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	double x=goal->pose.position.x;
	double y=goal->pose.position.y;
	double theta = tf::getYaw(goal->pose.orientation);
	

	if (utils::TF.transformPose(x,y,theta,goal->header.frame_id,"map")) {
		utils::Vector2d position(x,y);
		utils::RosMap::Pixel pixel;
		utils::MAP.mapToPixel(position,pixel);
		x_end = pixel.x;
		y_end = pixel.y;
		algorithm.start(x_init,y_init,x_end,y_end,x_max,y_max,d_min,delta_t,&ROS_OBSTACLES,robot_radius);

	}
	
}

void Node::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	double x=odom->pose.pose.position.x;
	double y=odom->pose.pose.position.y;
	double theta=tf::getYaw(odom->pose.pose.orientation);
	
	
	if (utils::TF.transformPose(x,y,theta,odom->header.frame_id,"map")) {
		utils::Vector2d position(x,y);
		utils::RosMap::Pixel pixel;
		utils::MAP.mapToPixel(position,pixel);
		x_init = pixel.x;
		y_init = pixel.y;
		

	}
	
	
}


void Node::drawRRT(Membrane *root,visualization_msgs::Marker& marker)
{
	utils::RosMap::Pixel p0,p1;
	geometry_msgs::Point m0,m1;
	p0.x = root->getX();
	p0.y = root->getY();
	utils::Vector2d x0,x1;
	utils::MAP.pixelToMap(p0,x0);
	m0.x = x0.getX();
	m0.y = x0.getY();
	for (int i=0;i<root->childsSize();i++) {
		p1.x = root->getChild(i)->getX();
		p1.y = root->getChild(i)->getY();
		utils::MAP.pixelToMap(p1,x1);
		m1.x = x1.getX();
		m1.y = x1.getY();
		marker.points.push_back(m0);
		marker.points.push_back(m1);
		drawRRT(root->getChild(i),marker);
	}
	
}


void Node::initMarker(visualization_msgs::Marker& marker)
{
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.id = 1;

	marker.color.r = 0;
	marker.color.g =0;
	marker.color.b =1.0;
	marker.color.a = 1.0;

	marker.action = 0; 
	marker.scale.x = 0.05;
	marker.type = 5;
	

}

void Node::drawRRT()
{
	visualization_msgs::Marker marker;
	
	initMarker(marker);

	if (algorithm.getRootA()!=NULL) {
	drawRRT(algorithm.getRootA(), marker);
	}
	if (algorithm.getRootB()!=NULL) {
	drawRRT(algorithm.getRootB(), marker);	
	}
	rrt_pub.publish(marker);
}

void Node::drawPath()
{
	visualization_msgs::Marker marker;
	initMarker(marker);
	marker.id=2;
	marker.color.r=1.0;
	marker.color.b=0.0;
	marker.scale.x = 0.2;
	marker.type = 4;
	geometry_msgs::Point p;
	utils::Vector2d v;
	utils::RosMap::Pixel pixel;
	for (auto it = algorithm.getPath().begin(); it!=algorithm.getPath().end();++it) {
		pixel.x = it->x;
		pixel.y = it->y;
		utils::MAP.pixelToMap(pixel,v); 
		p.x = v.getX();
		p.y = v.getY();
		marker.points.push_back(p);

	}
	path_pub.publish(marker);
}


Node::Node(ros::NodeHandle& n, ros::NodeHandle& pn)
{
	std::string goal_topic, odom_topic;	
	double freq;
	
	pn.param<std::string>("goal_topic",goal_topic,"/move_base_simple/goal");
	pn.param<std::string>("odom_topic",odom_topic,"/rosaria/pose");
	pn.param<double>("freq",freq,15);
	pn.param<double>("delta_t",delta_t,1.0); 
	pn.param<double>("d_min",d_min,0.05);
	pn.param<double>("robot_radius",robot_radius,0.3);


	//ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>(goal_topic,1,&Node::goalReceived,this);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_topic, 1, &Node::odomReceived,this);
	//rrt_pub = pn.advertise<visualization_msgs::Marker>("/renpsm/rrt", 1);	
	//path_pub = pn.advertise<visualization_msgs::Marker>("/renpsm/path", 1);	

	x_max =utils::MAP.getInfo().width;
	y_max =utils::MAP.getInfo().height;
	d_min = d_min/utils::MAP.getInfo().resolution;	
	robot_radius = robot_radius/utils::MAP.getInfo().resolution;	
	
	ros::Rate r(freq);
			
	int iter=0;
	double dis_min=9999;
	std::cout<<"Beginning benchmark:"<<std::endl;
	while (n.ok()) {

		r.sleep();
		ros::spinOnce();
		
		if (iter>30) {
			utils::Vector2d position(13.3,13.6);
			utils::RosMap::Pixel pixel;
			utils::MAP.mapToPixel(position,pixel);
			x_end = pixel.x;
			y_end = pixel.y;
			utils::Vector2d v,w;
			double dis=0;
			bool first=true;
			algorithm.start(x_init,y_init,x_end,y_end,x_max,y_max,d_min,delta_t,&ROS_OBSTACLES,robot_radius);
			
			
			for (int i=0;i<1000;i++) {
				algorithm.step();
				
			}
			for (auto it = algorithm.getPath().begin(); it!=algorithm.getPath().end();++it) {
				pixel.x = it->x;
				pixel.y = it->y;
				utils::MAP.pixelToMap(pixel,v);
				if (!first) {
					w = w - v;
					dis += w.norm();
				} else {
					first=false;
				}
				
				
				w=v; 
				
			}
			if (dis>0 && dis<dis_min) {
				dis_min = dis;
			}
			if ((iter-29)%10==0) {
				std::cout<<"Iteration "<<(iter-29)<<".- Cost: "<<dis<<" "<<"Min cost: "<<dis_min<<std::endl;
			}
			
			
		}
		//drawRRT();
		//drawPath();
		utils::MAP.computeDistances();	
		iter++;
	}

}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "renpsm");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	renpsm::Node node(n,pn);
	return 0;
}





