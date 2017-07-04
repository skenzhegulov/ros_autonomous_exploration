#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace tf;
using namespace ros;

typedef std::multimap<double, unsigned int> Queue;
typedef std::pair<double, unsigned int> Entry;

const unsigned int BLOCK_SIZE = 8;

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
	double getGainConst() { return mGainConst; }
	void setGainConst(double c) { mGainConst = c; }
	void setRobotRadius(double r) { mRobotRadius = r; }
	
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
		int minVal = 101;
		int maxVal = -1;
		int radius = ceil(1.3*mRobotRadius/getResolution()/100.0);
		int yCenter = index / mMapWidth;
		int xCenter = index % mMapWidth;
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
		
		ROS_DEBUG("Current cell index: %d", index);
		ROS_DEBUG("Current cell area minVal: %d", minVal);
		ROS_DEBUG("Current cell area maxVal: %d", maxVal);

		if(minVal >= 0 && maxVal < mLethalCost) return true;
		return false;
	}

	bool isFrontier(unsigned int index)
	{
		if(uFunction(index, ceil(mRobotRadius/getResolution()/45.0)) > mGainConst) return true;
/*
		if(getData(x-1, y-1) == -1) return true;
		if(getData(x-1, y  ) == -1) return true;
		if(getData(x-1, y+1) == -1) return true;
		if(getData(x  , y-1) == -1) return true;
		if(getData(x  , y+1) == -1) return true;
		if(getData(x+1, y-1) == -1) return true;
		if(getData(x+1, y  ) == -1) return true;
		if(getData(x+1, y+1) == -1) return true;
*/
		return false;

	}

	double uFunction(unsigned int index, unsigned int radius)
	{
		int xCenter = index / mMapWidth;
		int yCenter = index % mMapWidth;
		unsigned int current_index;
		std::map<unsigned int, double> minDistance;

		int all_cells = 0;
		for(int x = xCenter - radius; x <= xCenter; ++x)
			for(int y = yCenter - radius; y <= yCenter; ++y)
				if((x - xCenter)*(x - xCenter) + (y - yCenter)*(y - yCenter) <= radius*radius)
				{
					int xx = xCenter - (x - xCenter);
					int yy = yCenter - (y - yCenter);

					getIndex(x, y, current_index);
					minDistance[current_index] = radius*radius;
					
					getIndex(xx, y, current_index);
					minDistance[current_index] = radius*radius;
					
					getIndex(x, yy, current_index);
					minDistance[current_index] = radius*radius;
					
					getIndex(xx, yy, current_index);
					minDistance[current_index] = radius*radius;

				}

		Queue queue;
		queue.insert(Entry(0, index));
		minDistance[index] = 0;
		
		int use_cells = 0;

		Queue::iterator next;
		while(queue.size()) {
			next = queue.begin();
			current_index = next->second;
			double distance = next->first;
			
			queue.erase(next);

			all_cells++;

			int value = getData(current_index);
			if(value < mLethalCost) 
			{
				unsigned int ind[4];
				ind[0] = current_index - 1;
				ind[1] = current_index + 1;
				ind[2] = current_index - mMapWidth;
				ind[3] = current_index + mMapWidth;

				for(int i=0; i<4; ++i)
					if(minDistance.find(ind[i]) != minDistance.end()
					&& minDistance[ind[i]] > distance + 1)
					{
						minDistance[ind[i]] = distance + 1;
						queue.insert(Entry(distance + 1, ind[i]));
					}
			}

			if(value) use_cells++;
		}
		double gain = 100.0*use_cells / (all_cells*1.0);
		ROS_INFO("uFunction value: %f", gain);
		return gain;
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
	double mRobotRadius;
	char mLethalCost;
	double mGainConst;
};           

#endif
