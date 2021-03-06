#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace tf;
using namespace ros;

typedef std::multimap<double, unsigned int> Queue;
typedef std::pair<double, unsigned int> Entry;

class GridMap
{
public:    
	void generateMap() {
		boost::posix_time::ptime current_time = Time::now().toBoost();
		std::string name = boost::posix_time::to_iso_extended_string(current_time);
		std::string mapdatafile = mPath+name+".pgm";
		ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
		FILE* out = fopen(mapdatafile.c_str(), "w");
		if(!out)
		{
			ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
			return;
		}

		const nav_msgs::OccupancyGrid& map = getMap();

		fprintf(out, "P5\n# Generated map %.3f m/pix\n%d %d\n255\n",
				map.info.resolution, map.info.width, map.info.height);
		for(unsigned int y = 0; y < map.info.height; ++y) 
		{
			for(unsigned int x = 0; x < map.info.width; ++x)
			{
				unsigned int i = x + (map.info.height - y - 1) * map.info.width;
				if(map.data[i] == 0)
				{
					fputc(254, out);
				} 
				else 
				if(map.data[i] >= mLethalCost)
				{
					fputc(000, out);
				}
				else
				{
					fputc(205, out);
				}
			}
		}

		fclose(out);

		std::string mapmetadatafile = mPath+name+".yaml";
		ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
		FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

		geometry_msgs::Quaternion orientation = map.info.origin.orientation;
		tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
		double yaw, pitch, roll;
		mat.getEulerYPR(yaw, pitch, roll);

		fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_tresh: 0.196\n\n",
				mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

		fclose(yaml);

		ROS_INFO("Done\n");
	}

	bool getCurrentPosition(unsigned int &index) 
	{
		TransformListener mTfListener_;
	    try
	    {
		   	Time now = Time::now();
		   	mTfListener_.waitForTransform(std::string("map"), 
										  std::string("base_footprint"), 
										  Time(0), Duration(10.0));

	        tf::StampedTransform transform;
		   	mTfListener_.lookupTransform(std::string("map"), 
										 std::string("base_footprint"), 
										 Time(0), transform);

	    	double x = transform.getOrigin().x();
	    	double y = transform.getOrigin().y();
	    	double w = tf::getYaw(transform.getRotation());

	    	unsigned int X = (x - getOriginX()) / getResolution();
	    	unsigned int Y = (y - getOriginY()) / getResolution();
	
	    	if(!getIndex(X, Y, index))
	    	{
		   		ROS_ERROR("Is the robot out of the map?");
		   		return false;
	    	}
	    			
			ROS_INFO("Robot's coordinates are %d, %d \n", X, Y);
	    }
	    catch(TransformException ex)
	    {
			ROS_ERROR("Could not get robot position: %s", ex.what());
		   	return false;
		}

	    return true;
    }


	bool getIndex(unsigned int x, unsigned int y, unsigned int &i)
	{
		if(x >= mMapWidth || y >= mMapHeight)
		{
			return false;
		}
		
		i = y*mMapWidth + x;
		return true;
	}

	bool getCoordinates(unsigned int &x, unsigned int &y, unsigned int i)
	{
		if(i >= mMapWidth*mMapHeight)
		{
			ROS_ERROR("getCoords() failed!");
			return false;
		}
		
		y = i/mMapWidth;
		x = i%mMapWidth;
		return true;
	}

	bool getOdomCoordinates(double &x, double &y, unsigned int i) {
		unsigned int X;
		unsigned int Y;
		if(!getCoordinates(X, Y, i)) {
			return false;
		}
		x = X*getResolution() + getOriginX();
        y = Y*getResolution() + getOriginY();
		return true;
	}

	void clearArea(unsigned int center) {
		unsigned int xCenter;
		unsigned int yCenter;
		getCoordinates(xCenter, yCenter, center);

		int radius = ceil(mRobotRadius / getResolution() / 100.0);

		int vx[2], vy[2];
		for(int x = xCenter - radius; x <= xCenter; ++x)
			for(int y = yCenter - radius; y <= yCenter; ++y)
				if((x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter) <= radius*radius) 
				{
					vx[0] = x;
					vy[0] = y;
					vx[1] = xCenter - (x - xCenter);
					vy[1] = yCenter - (y - yCenter);

					for(int i=0; i<2; ++i)
						for(int j=0; j<2; ++j)
							{
								setData(vx[i], vy[j], 0);
							}
				}

	}

	bool isFree(unsigned int xCenter, unsigned int yCenter)
	{
		int minVal = 101;
		int maxVal = -1;
		int radius = ceil(1.3*mRobotRadius / getResolution());
		int val, vx[2], vy[2];
		for(int x = xCenter - radius; x <= xCenter; ++x)
			for(int y = yCenter - radius; y <= yCenter; ++y)
				if((x!=xCenter || y!=yCenter) && (x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter) <= radius*radius) 
				{
					vx[0] = x;
					vy[0] = y;
					vx[1] = xCenter - (x - xCenter);
					vy[1] = yCenter - (y - yCenter);

					for(int i=0; i<2; ++i)
						for(int j=0; j<2; ++j)
							{
								val = getData(vx[i],vy[j]);
								minVal = std::min(minVal, val);
								maxVal = std::max(maxVal, val);
							}
				}
		
		ROS_DEBUG("Current cell area minVal: %d", minVal);
		ROS_DEBUG("Current cell area maxVal: %d", maxVal);

		if(maxVal < mLethalCost && minVal > -1) return true;
		return false;
	}

	bool isFree(unsigned int index)
	{
		unsigned int x;
		unsigned int y;
	    getCoordinates(x, y, index);
		return isFree(x, y);
	}

	bool isFrontier(unsigned int X, unsigned int Y) 
	{
		ROS_DEBUG("Map resolution: %f", getResolution());
		unsigned int index;
		getIndex(X, Y, index);
		return uFunction(index) > mGainConst;	
	}

	bool isFrontier(unsigned int index)
	{
		ROS_DEBUG("Map resolution: %f", getResolution());
		return uFunction(index) > mGainConst;
	}

	double uFunction(unsigned int index)
	{
		unsigned int xCenter;
		unsigned int yCenter;
		getCoordinates(xCenter, yCenter, index);
		unsigned int radius = ceil(mLaserRange / getResolution() / 2.0);

		unsigned int current_index;
		std::map<unsigned int, double> minDistance;

		ROS_DEBUG("Area radius: %d", radius);

		int all_cells = 3*radius*radius;
		for(int x = xCenter; x <= xCenter + radius; ++x)
			for(int y = yCenter; y <= yCenter + radius; ++y)
				if((x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter) <= radius*radius)
				{
					int xx = xCenter - (x - xCenter);
					int yy = yCenter - (y - yCenter);

					if(getIndex(x, y, current_index))
					{
						minDistance[current_index] = radius*radius*radius;
					}	
					if(getIndex(xx, y, current_index))
					{
						minDistance[current_index] = radius*radius*radius;
					}
					if(getIndex(x, yy, current_index))
					{
						minDistance[current_index] = radius*radius*radius;
					}
					if(getIndex(xx, yy, current_index))
					{
						minDistance[current_index] = radius*radius*radius;
					}
				}

		Queue queue;
		queue.insert(Entry(0.0, index));
		minDistance[index] = 0.0;
		
		int use_cells = 0;

		Queue::iterator next;
		while(queue.size()) {
			next = queue.begin();
			current_index = next->second;
			double distance = next->first;
			
			queue.erase(next);

			int value = getData(current_index);
			unsigned int x, y;
			getCoordinates(x, y, current_index);

			ROS_DEBUG("Current cell %d value: %d", current_index, value);

			if(value < mLethalCost && distance <= std::max(std::abs(x-xCenter), std::abs(y-yCenter))) 
			{
				unsigned int ind[8];
				ind[0] = current_index + 1;
				ind[1] = current_index - 1;
				ind[2] = current_index + mMapWidth;
				ind[3] = current_index - mMapWidth;
				ind[4] = current_index + mMapWidth + 1;
				ind[5] = current_index + mMapWidth - 1;
				ind[6] = current_index - mMapWidth + 1;
				ind[7] = current_index - mMapWidth - 1;

				for(int i=0; i<8; ++i)
				{
					if(minDistance.find(ind[i]) != minDistance.end()
					&& minDistance[ind[i]] > distance + 1)
					{
						minDistance[ind[i]] = distance + 1;
						queue.insert(Entry(distance + 1, ind[i]));
					}
				}

			    if(value > 0) use_cells++;
			}

		}
		double gain = 100.0*use_cells / (all_cells*1.0);
		ROS_DEBUG("uFunction value: %f", gain);
		ROS_DEBUG("Useful cells: %d", use_cells);
		ROS_DEBUG("All counted cells: %d", all_cells);
		return gain;
	}

	char getData(unsigned int index)
	{
		if(index < mMapWidth*mMapHeight)
			return mOccupancyGrid.data[index];
		else
			return -1;
	}

	char getData(int x, int y)
	{
		if(x < 0 || x >= (int)mMapWidth || y < 0 || y >= (int)mMapHeight) return -1;
		
		return mOccupancyGrid.data[y*mMapWidth + x];
	}

	bool setData(unsigned int index, char value)
	{
		if(index >= mMapWidth*mMapHeight)
		{
			return false;
		}

		mOccupancyGrid.data[index] = value;
		return true;
	}

	bool setData(int x, int y, char value)
	{
		if(x < 0 || x >= (int)mMapWidth || y < 0 || y >= (int)mMapHeight) return false;

		mOccupancyGrid.data[y*mMapWidth + x] = value;
		return true;
	}

	void update(nav_msgs::OccupancyGrid grid)
	{
		mOccupancyGrid = grid;
		mMapWidth = mOccupancyGrid.info.width;
		mMapHeight = mOccupancyGrid.info.height;
		ROS_DEBUG("Got new map of size %d x %d", mMapWidth, mMapHeight);
	}

	const nav_msgs::OccupancyGrid& getMap() const { return mOccupancyGrid; }

	unsigned int getWidth() { return mMapWidth; }
	unsigned int getHeight() { return mMapHeight; }
	unsigned int getSize() { return mMapWidth * mMapHeight; }
	double getResolution() { return mOccupancyGrid.info.resolution; }
	double getOriginX() { return mOccupancyGrid.info.origin.position.x; }
	double getOriginY() { return mOccupancyGrid.info.origin.position.y; }

	char getLethalCost() { return mLethalCost; }
	void setLethalCost(char c) { mLethalCost = c; };
	
	double getGainConst() { return mGainConst; }
	void setGainConst(double c) { mGainConst = c; }
	
	double getRobotRadius() { return mRobotRadius; }
	void setRobotRadius(double r) { mRobotRadius = r; }

	double getLaserRange() { return mLaserRange; }
	void setLaserRange(double r) { mLaserRange = r; }
	
	std::string getPath() { return mPath; }
	void setPath(std::string s) { mPath = s; }
	

private:
	nav_msgs::OccupancyGrid mOccupancyGrid;
	unsigned int mMapWidth;
	unsigned int mMapHeight;
	double mRobotRadius;
	double mLaserRange;
	char mLethalCost;
	double mGainConst;
	std::string mPath;
};           

#endif
