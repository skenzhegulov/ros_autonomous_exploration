#include <ros/ros.h>
#include <map>
#include <std_msgs/Float64.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <boost/thread.hpp>

using namespace ros;

nav_msgs::GetMap::Response map_;
boost::mutex map_mutex;
bool got_map;

void mapUpdate(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	ROS_DEBUG("Updating a map");
	boost::mutex::scoped_lock map_lock (map_mutex);

	map_.map.info.width = map->info.width;
	map_.map.info.height = map->info.height;
	map_.map.info.resolution = map->info.resolution;

	map_.map.info.map_load_time = map->info.map_load_time;

	map_.map.info.origin.position.x = map->info.origin.position.x;
	map_.map.info.origin.position.y = map->info.origin.position.y;
	map_.map.info.origin.position.z = map->info.origin.position.z;
	
	map_.map.info.origin.orientation.x = map->info.origin.orientation.x;
	map_.map.info.origin.orientation.y = map->info.origin.orientation.y;
	map_.map.info.origin.orientation.z = map->info.origin.orientation.z;

	map_.map.data.resize(map->info.width*map->info.height);

	std::map<int, int> mp;
	for(int i=0; i<map->data.size(); i++) 
		mp[map->data[i]]++,
		map_.map.data[i] = map->data[i];

	map_.map.header.stamp = map->header.stamp;
	map_.map.header.frame_id = map->header.frame_id;
/*
	ROS_INFO("Got map!");
	for(std::map<int, int>::iterator it=mp.begin(); it!=mp.end(); it++)
		ROS_INFO("%d, %d", it->first, it->second);
*/
	got_map = true;
}


bool mapCallback(nav_msgs::GetMap::Request  &req, 
		 nav_msgs::GetMap::Response &res)
{
	boost::mutex::scoped_lock map_lock (map_mutex);
	if(got_map && map_.map.info.width && map_.map.info.height)
	{
		res = map_;
		return true;
	}
	else
		return false;
}

int main(int argc, char **argv)
{
	init(argc, argv, "map_server");
	NodeHandle n;

	Subscriber sub = n.subscribe("map", 10, mapUpdate);

	ServiceServer ss = n.advertiseService("current_map", mapCallback);

	spin();

	return 0;
}
