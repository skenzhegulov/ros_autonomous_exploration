#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace tf;
using namespace ros;

const unsigned int BLOCK_SIZE = 30;

class GridMap
{
public:    
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

	int convertToBlock(int index) 
    	{
		int x = index%mMapWidth;
		if(!x) x+=mMapWidth;
		x = x/BLOCK_SIZE + (x%BLOCK_SIZE ? 1 : 0);
		
		int y = (index-1)/(mMapWidth*BLOCK_SIZE);
		return x + y*mBlockWidth;
    	}

	void update(nav_msgs::OccupancyGrid grid)
	{
		mOccupancyGrid = grid;
		mMapWidth = mOccupancyGrid.info.width;
		mMapHeight = mOccupancyGrid.info.height;
		mBlockWidth = mMapWidth/BLOCK_SIZE + (mMapWidth%BLOCK_SIZE ? 1 : 0);
		mBlockHeight = mMapHeight/BLOCK_SIZE + (mMapHeight%BLOCK_SIZE ? 1 : 0);
		ROS_DEBUG("Got new map of size %d x %d", mMapWidth, mMapHeight);
		ROS_DEBUG("Block dimensions %d x %d", mBlockWidth, mBlockHeight);
	}

	unsigned int getWidth() { return mMapWidth; }
	unsigned int getHeight() { return mMapHeight; }
	unsigned int getSize() { return mMapWidth * mMapHeight; }
	unsigned int getBlockWidth() { return mBlockWidth; }
	unsigned int getBlockHeight() { return mBlockHeight; }
	unsigned int getBlockSize() { return mBlockWidth * mBlockHeight; }
	double getResolution() { return mOccupancyGrid.info.resolution; }
	double getOriginX() { return mOccupancyGrid.info.origin.position.x; }
	double getOriginY() { return mOccupancyGrid.info.origin.position.y; }

	char getLethalCost() { return mLethalCost; }
	void setLethalCost(char c) { mLethalCost = c; };
	
	const nav_msgs::OccupancyGrid& getMap() const { return mOccupancyGrid; }

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

	char getData(unsigned int index)
	{
		if(index < mMapWidth*mMapHeight)
			return mOccupancyGrid.data[index];
		else
			return -1;
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

	bool isFree(unsigned int index)
	{
		char value = getData(index);
		if(value >= 0 && value < mLethalCost) return true;
		return false;
	}

	bool isFrontier(unsigned int index)
	{
		int y = index / mMapWidth;
		int x = index % mMapWidth;

		if(getData(x-1, y-1) == -1) return true;
		if(getData(x-1, y  ) == -1) return true;
		if(getData(x-1, y+1) == -1) return true;
		if(getData(x  , y-1) == -1) return true;
		if(getData(x  , y+1) == -1) return true;
		if(getData(x+1, y-1) == -1) return true;
		if(getData(x+1, y  ) == -1) return true;
		if(getData(x+1, y+1) == -1) return true;

		return false;
	}

	char getData(int x, int y)
	{
		if(x < 0 || x >= (int)mMapWidth || y < 0 || y >= (int)mMapHeight) return -1;
		
		return mOccupancyGrid.data[y*mMapWidth + x];
	}

	bool setData(int x, int y, char value)
	{
		if(x < 0 || x >= (int)mMapWidth || y < 0 || y >= (int)mMapHeight) return false;

		mOccupancyGrid.data[y*mMapWidth + x] = value;
		return true;
	}

	bool isFree(int x, int y)
	{
		char value = getData(x, y);
		if(value >= 0 && value < mLethalCost) return true;

		return false;
	}

private:
	nav_msgs::OccupancyGrid mOccupancyGrid;
	unsigned int mMapWidth;
	unsigned int mMapHeight;
	unsigned int mBlockWidth;
	unsigned int mBlockHeight;
	char mLethalCost;
};           

#endif
