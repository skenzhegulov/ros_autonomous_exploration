#include <ros/ros.h>
#include <map>
#include <queue>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <autonomous_exploration/ExploreAction.h>
#include <autonomous_exploration/GridMap.h>
#include <autonomous_exploration/VisMarker.h>
#include <autonomous_exploration/PoseWrap.h>
#include <autonomous_exploration/Color.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace tf;
using namespace ros;

class ExploreAction {

public:

   	ExploreAction(std::string name) : 
		as_(nh_, name, boost::bind(&ExploreAction::run, this, _1), false),
		action_name_(name),
		ac_("move_base", true)
	{
		mMarker_pub_ = nh_.advertise<visualization_msgs::Marker>("vis_marker", 2);
		mGetMapClient_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("current_map"));

		int lethal_cost;
		nh_.param("lethal_cost", lethal_cost, 85);
		ROS_INFO("Param lethal_cost: %d", lethal_cost);
		mCurrentMap_.setLethalCost(lethal_cost);

		double gain_const;
		nh_.param("gain_const", gain_const, 23.);
		ROS_INFO("Param gain_const: %f", gain_const);
		mCurrentMap_.setGainConst(gain_const);
		
		double robot_radius;
		nh_.param("robot_radius", robot_radius, 18.);
		ROS_INFO("Param robot_radius: %f", robot_radius);
		mCurrentMap_.setRobotRadius(robot_radius);

		std::string map_path;
		std::string temp = "/";
		nh_.param("map_path", map_path, temp);
		ROS_INFO("Map directory path: %s", map_path.c_str());
		mCurrentMap_.setPath(map_path);

		as_.registerPreemptCallback(boost::bind(&ExploreAction::preemptCB, this));

		as_.start();

	}

	~ExploreAction(void) 
	{
	}

    void run(const autonomous_exploration::ExploreGoalConstPtr &goal)
	{
		Rate loop_rate(2);

		double currentGain = mCurrentMap_.getGainConst();
		int count = 0;

		while(ok() && as_.isActive() && currentGain > 8.0)
		{
			loop_rate.sleep();
			mCurrentMap_.setGainConst(currentGain);
			
			unsigned int pos_index;

			if(!getMap()) 
			{
				ROS_ERROR("Could not get a map");
				as_.setPreempted();
				return;
			}

			if(!mCurrentMap_.getCurrentPosition(pos_index))
			{
				ROS_ERROR("Could not get a position");
				as_.setPreempted();
				return;
			}

			int explore_target = explore(&mCurrentMap_, pos_index);

			if(explore_target != -1) 
			{
				moveTo(explore_target);

				if(count) 
				{
					currentGain *= 1.5;
					count--;
					ROS_INFO("Gain was increased to: %f", currentGain);
				}
			}
			else
			{
				currentGain /= 1.5;
				count++;
				ROS_INFO("Gain was decreased to: %f", currentGain);
			}
		}

		if(as_.isActive()) 
		{
			as_.setSucceeded(result_);
			ROS_INFO("Exploration finished");
		}
		else
		{
			ROS_INFO("Exploration was interrupted");
		}
    }

	void preemptCB() 
	{
		ROS_INFO("Server received a cancel request.");
		mCurrentMap_.generateMap();
		goal_reached = 2;
		ac_.cancelGoal();
		as_.setPreempted();
	}    
	
	void doneCb(const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResult::ConstPtr& result)
    {
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Robot reached the explore target");
			goal_reached = 1;
		} 
		else
		{
			goal_reached = -1;
		}
    }

private:
    int explore(GridMap *map, unsigned int start)
    {
	    ROS_INFO("Starting exploration");

		map->clearArea(start);
		ROS_INFO("Cleared the area");

	    unsigned int mapSize = map->getSize();
	    double *plan = new double[mapSize];
	    for(unsigned int i = 0; i<mapSize; i++)
	    {
		    plan[i] = -1;
	    }

	    Queue queue;
	    Entry startPoint(0.0, start);
	    queue.insert(startPoint);
	    plan[start] = 0;

	    Queue::iterator next;
	    double distance;
	    double linear = map->getResolution();
	    int foundFrontier = 0;

	    while(!queue.empty() && !foundFrontier) 
	    {
		    next = queue.begin();
		    distance = next->first;
		    unsigned int index = next->second;
		    queue.erase(next);

			ROS_DEBUG("Target distance: %f", distance);
		    if(distance > 16*linear && map->isFrontier(index))
		    {
				foundFrontier = index;
				publishMarker(index, 2);
		    }
		    else
		    {
			    unsigned int ind[4];

			    ind[0] = index - 1;
			    ind[1] = index + 1;
			    ind[2] = index - map->getWidth();
			    ind[3] = index + map->getWidth();

			    for(unsigned int it = 0; it<4; ++it)
			    {
				    unsigned int i = ind[it];
				    if(map->isFree(i) && plan[i] == -1) 
				    {
					    queue.insert(Entry(distance+linear, i));
					    plan[i] = distance+linear;

						publishMarker(index, 1);
				    }
			    }
		    }
	    }

	    delete [] plan;
		queue.clear();

	    if(foundFrontier) 
	    {
		    return foundFrontier;
	    }
	    else
	    {
			return -1;
	    }
    }

    bool moveTo(unsigned int goal_index)
    {
		Rate loop_rate(2);

		while(!ac_.waitForServer()) 
		{
			ROS_INFO("Waiting for the move_base action server to come up");
			loop_rate.sleep();
		}

		double x, y;
		mCurrentMap_.getOdomCoordinates(x, y, goal_index);
	
		ROS_INFO("Got the explore goal: %f, %f\n", x, y);

	   	move_base_msgs::MoveBaseGoal move_goal;

		move_goal.target_pose.header.frame_id = "map";
		move_goal.target_pose.header.stamp = Time(0);

		PoseWrap pose(x, y);
		move_goal.target_pose.pose = pose.getPose();
		
		ROS_INFO("Sending goal");
		ac_.sendGoal(move_goal,
			     boost::bind(&ExploreAction::doneCb, this, _1, _2));
		
		goal_reached = 0;

		while(goal_reached == 0 && ok()) 
		{
			loop_rate.sleep();
		}
		
		if(goal_reached == 1) 
		{
			ROS_INFO("Reached the goal");
			return true;
		}   
		
		ROS_INFO("Failed to reach the goal");	
		return false;
    }

    bool getMap() 
    {
	    if(!mGetMapClient_.isValid()) 
		{
		    return false;
		}

	    nav_msgs::GetMap srv;

	    if(!mGetMapClient_.call(srv)) 
	    {
		    ROS_INFO("Could not get a map.");
		    return false;
	    }

	    mCurrentMap_.update(srv.response.map);
	    ROS_INFO("Got new map of size %d x %d", mCurrentMap_.getWidth(), mCurrentMap_.getHeight());
	    ROS_INFO("Map resolution is: %f", mCurrentMap_.getResolution());

	    return true;
    }

	void publishMarker(unsigned int index, int type) {
		double x, y;
		mCurrentMap_.getOdomCoordinates(x, y, index);
		PoseWrap pose(x, y);

		VisMarker marker;
		/*
		  type:
			1: free area
			2: frontier
		*/
		if(type == 1) {
			Color color(0, 0, 0.7);
			marker.setParams("free", pose.getPose(), 0.35, color.getColor());
		} else
		if(type == 2) {
			Color color(0, 0.7, 0);
			marker.setParams("frontier", pose.getPose(), 0.75, color.getColor());
		}

		mMarker_pub_.publish(marker.getMarker());
	}	

protected:
    NodeHandle nh_;
	actionlib::SimpleActionServer<autonomous_exploration::ExploreAction> as_;
	autonomous_exploration::ExploreFeedback feedback_;
	autonomous_exploration::ExploreResult result_;
	std::string action_name_;
	MoveBaseClient ac_;

    ServiceClient mGetMapClient_;
    GridMap mCurrentMap_;
    int goal_reached;

	Publisher mMarker_pub_;
};

int main(int argc, char **argv) 
{
	init(argc, argv, "explore_server_node");

	ExploreAction explore("explore");
	
	spin();
	
	return 0;
}
