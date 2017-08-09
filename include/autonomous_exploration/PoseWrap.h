#ifndef POSE_WRAP_H
#define POSE_WRAP_H

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

using namespace ros;

class PoseWrap
{
public:
	PoseWrap(double x, double y, double w = 1.) {
		pose.position.x = x;
		pose.position.y = y;
		pose.orientation.w = w;
	}

	geometry_msgs::Pose getPose() {
		return pose;
	}

protected:
	geometry_msgs::Pose pose;
};

#endif

