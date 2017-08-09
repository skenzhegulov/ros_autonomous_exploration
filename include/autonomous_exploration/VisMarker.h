#ifndef VIS_MARKER_H
#define VIS_MARKER_H

#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/ColorRGBA.h>

using namespace ros;

class VisMarker
{
public:
	VisMarker() {
		marker.header.frame_id = "map";
        marker.header.stamp = Time::now();
        marker.ns = "none";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = 0.2;
        marker.id = 0;
		marker.color.a = 1.0; 		
	}

	void setParams(std::string ns, geometry_msgs::Pose pose, double scale, std_msgs::ColorRGBA color, int type = 3) {
		marker.ns = ns;
		marker.pose = pose;
		marker.scale.x = marker.scale.y = scale;
		marker.color = color;
		marker.type = type;
	}

	visualization_msgs::Marker getMarker() {
		return marker;
	}

protected:
	visualization_msgs::Marker marker;
};

#endif

