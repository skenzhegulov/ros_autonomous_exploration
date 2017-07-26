#include <ros/ros.h>
#include <map>
#include <queue>
#include <utility>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <autonomous_exploration/ExploreAction.h>
#include <autonomous_exploration/GridMap.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace tf;
using namespace ros;

class ExploreAction {

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
   
public:

   	ExploreAction(std::string name) : 
		as_(nh_, name, boost::bind(&ExploreAction::run, this, _1), false),
		action_name_(name),
		ac_("move_base", true)
	{
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
		ROS_INFO("Running exploration server");

		unsigned int explore_target = 0;

		while(explore_target != -1 && ok() && as_.isActive())
		{
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

			explore_target = explore(&mCurrentMap_, pos_index);

			if(explore_target != -1) 
			{
				moveTo(explore_target);
			}
		}

		if(explore_target == -1)
		{
			ROS_ERROR("Could not get a explore target: %d", explore_target);
			as_.setSucceeded(result_);
		}
    }

    int explore(GridMap *map, unsigned int start)
    {
	    ROS_INFO("Starting exploration");
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

			ROS_INFO("Target distance: %f", distance);
		    if(distance > 16*linear && map->isFrontier(index))
		    {
				foundFrontier = index;
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
				    }
			    }
		    }
	    }

	    delete [] plan;

	    if(foundFrontier) 
	    {
		    return foundFrontier;
	    }
	    else
	    {
			return -1;
	    }
    }

	void preemptCB() 
	{
		ROS_INFO("Server received a cancel request.");
		mCurrentMap_.generateMap();
		ac_.cancelGoal();
		as_.setPreempted();
	}

    bool moveTo(unsigned int goal_index)
    {
		while(!ac_.waitForServer()) 
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}

		unsigned int X;
		unsigned int Y;
		
		mCurrentMap_.getCoordinates(X, Y, goal_index);
		
		double x = X*mCurrentMap_.getResolution() + mCurrentMap_.getOriginX();	
		double y = Y*mCurrentMap_.getResolution() + mCurrentMap_.getOriginY();
	
		geometry_msgs::Point p;
		p.x = x; p.y = y; p.z = 0.0;		
		
		feedback_.target = p;
		
		ROS_INFO("Got the explore goal: %d, %d\n", X, Y);
		ROS_INFO("Got the explore goal: %f, %f\n", x, y);

	   	move_base_msgs::MoveBaseGoal move_goal;

		move_goal.target_pose.header.frame_id = "map";
		move_goal.target_pose.header.stamp = Time(0);

		move_goal.target_pose.pose.position.x = x;
		move_goal.target_pose.pose.position.y = y;
		move_goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		ac_.sendGoal(move_goal,
			     boost::bind(&ExploreAction::doneCb, this, _1, _2));
		
		Rate loop_rate(2);

		goal_reached = 0;

		while(goal_reached == 0 && ok()) 
		{
			loop_rate.sleep();
		}
		
		result_.target = p;

		if(goal_reached == 1) 
		{
			ROS_INFO("Reached the goal");
			return true;
		}
		ROS_INFO("Failed to reach the goal");	
		return false;
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
      
};

int main(int argc, char **argv) 
{
	init(argc, argv, "explore_server_node");

	ExploreAction explore("explore");
	
	spin();
	
	return 0;
}
