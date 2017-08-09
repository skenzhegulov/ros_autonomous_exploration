#ifndef COLOR_H
#define COLOR_H

#include<ros/ros.h>
#include<std_msgs/ColorRGBA.h>

using namespace ros;

class Color
{
public:
	Color(double r, double g, double b, double a = 1.) {
		color.r = r;
		color.g = g;
		color.b = b;
		color.a = a;
	}

	std_msgs::ColorRGBA getColor() {
		return color;
	}

protected:
	std_msgs::ColorRGBA color;
};

#endif

