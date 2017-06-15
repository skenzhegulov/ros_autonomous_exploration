#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <exploration_target/ExploreAction.h>
#include <exploration_target/GridMap.h>
#include <exploration_target/ExploreTarget.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef std::multimap<double, unsigned int> Queue;
typedef std::pair<double, unsigned int> Entry;

using namespace tf;
using namespace ros;

class ExploreService {

protected:
	NodeHandle nh_;
	std::string action_name_;
	MoveBaseClient ac_;
	ServiceServer exploreServer_, cancelExploreServer_;
    ServiceClient mGetMapClient_, clearCostmapsClient_;
    GridMap mCurrentMap_;
    int goal_reached;
	bool interrupted;
   
public:

   	ExploreService() : 
		ac_("move_base", true)
	{
		while(!ac_.waitForServer()) 
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		interrupted = false;
		mCurrentMap_.setLethalCost(80);
		exploreServer_ = nh_.advertiseService("explore_service", &ExploreService::run, this);
		cancelExploreServer_ = nh_.advertiseService("explore_cancel", &ExploreService::cancel, this);
		mGetMapClient_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("current_map"));
	}

	~ExploreService(void) 
	{
	}

    bool run(exploration_target::ExploreTarget::Request& req,
			 exploration_target::ExploreTarget::Response& res)
	{
		ROS_INFO("Running exploration service");

		unsigned int pos_index;

		if(!getMap()) 
		{
			ROS_ERROR("Could not get a map");
			return false;
		}
			
		if(!mCurrentMap_.getCurrentPosition(pos_index))
		{
			ROS_ERROR("Could not get a position");
			return false;
		}

		explore(&mCurrentMap_, pos_index);
/*
		if(explore_state == 2) {
			ROS_ERROR("Exploration was interrupted");
			return false;
		}
		if(explore_state)
		{
			ROS_ERROR("Could not get a explore target: %d", explore_state);
			return false;
		}
*/
		return true;
    }

	bool cancel(exploration_target::ExploreTarget::Request& req,
				exploration_target::ExploreTarget::Response& res)
	{
		ROS_INFO("Cancel exploration request was sent.");

		interrupted = true;

		Publisher cancel_move = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
		actionlib_msgs::GoalID msg;
		cancel_move.publish(msg);	


		return true;
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

    bool moveTo(unsigned int goal_index)
    {		
		/*clearCostmapsClient_ = nh_.serviceClient<std_srvs::Empty>(std::string("move_base/clear_costmaps"));
		std_srvs::Empty srv;
		if(clearCostmapsClient_.call(srv)) 
			ROS_INFO("Costmaps are cleared successfully");
		else
			ROS_INFO("Failed to clear costmaps");*/

		unsigned int X;
		unsigned int Y;
		mCurrentMap_.getCoordinates(X, Y, goal_index);
		double x = X*mCurrentMap_.getResolution() + mCurrentMap_.getOriginX();	
		double y = Y*mCurrentMap_.getResolution() + mCurrentMap_.getOriginY();		
		
		ROS_INFO("Got the explore goal: %d, %d\n", X, Y);
		ROS_INFO("Got the explore goal: %f, %f\n", x, y);

	   	move_base_msgs::MoveBaseGoal move_goal;

		move_goal.target_pose.header.frame_id = "map";
		move_goal.target_pose.header.stamp = Time(0);

		move_goal.target_pose.pose.position.x = x;
		move_goal.target_pose.pose.position.y = y;
		move_goal.target_pose.pose.position.z = 0.0;
		move_goal.target_pose.pose.orientation.x = 0.0;
		move_goal.target_pose.pose.orientation.y = 0.0;
		move_goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		ac_.sendGoal(move_goal,
			     boost::bind(&ExploreService::doneCb, this, _1, _2));
		
		Rate loop_rate(1);

		goal_reached = 0;
		int count = 0;
		while(!goal_reached && !interrupted) 
		{
			count++;
			//loop_rate.sleep();
		}

		if(goal_reached == 1) 
		{
			ROS_INFO("Reached the goal");
			return true;
		}
		if(count > 200000)
		{
			ROS_INFO("Robot moved but couldn't reach the goal");
			return true;
		}
		ROS_INFO("Failed to reach the goal");	
		return false;
    }

    bool checkPath(unsigned int start, unsigned int goal) 
    {
		unsigned int X; double x;
		unsigned int Y; double y;
		mCurrentMap_.getCoordinates(X, Y, start);
		x = X*mCurrentMap_.getResolution() + mCurrentMap_.getOriginX();	
		y = Y*mCurrentMap_.getResolution() + mCurrentMap_.getOriginY();
	
		geometry_msgs::PoseStamped Start;
		Start.header.seq = 0;
		Start.header.stamp = Time(0);
		Start.header.frame_id = "map";
		Start.pose.position.x = x;
		Start.pose.position.y = y;
		Start.pose.position.z = 0.0;
		Start.pose.orientation.x = 0.0;
		Start.pose.orientation.y = 0.0;
		Start.pose.orientation.w = 1.0;		
		
		mCurrentMap_.getCoordinates(X, Y, goal);
		x = X*mCurrentMap_.getResolution() + mCurrentMap_.getOriginX();	
		y = Y*mCurrentMap_.getResolution() + mCurrentMap_.getOriginY();

		geometry_msgs::PoseStamped Goal;
		Goal.header.seq = 1;
		Goal.header.stamp = Time(0);
		Goal.header.frame_id = "map";
		Goal.pose.position.x = x;
		Goal.pose.position.y = y;
		Goal.pose.position.z = 0.0;
		Goal.pose.orientation.x = 0.0;
		Goal.pose.orientation.y = 0.0;
		Goal.pose.orientation.w = 1.0;

		ServiceClient check_path = nh_.serviceClient<nav_msgs::GetPlan>("make_plan");
		nav_msgs::GetPlan srv;
		srv.request.start = Start;
		srv.request.goal = Goal;
		srv.request.tolerance = 0.8;

		ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
		return false; 
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

	    unsigned int usedSize = map->getBlockSize();
	    ROS_INFO("Number of Blocks: %d", usedSize);
	    bool *used = new bool[usedSize];
	    for(unsigned int i = 0; i<usedSize; ++i)
	    {
			used[i] = false;
	    }

	    Queue queue;
	    Entry startPoint(0.0, start);
	    queue.insert(startPoint);
	    plan[start] = 0;
	    used[map->convertToBlock(start)] = true;

	    Queue::iterator next;
	    double distance;
	    double linear = map->getResolution();
	    bool foundFrontier = false;
	    int cellCount = 0;

	    while(!queue.empty() && !foundFrontier && !interrupted) 
	    {
		    cellCount++;
		    next = queue.begin();
		    distance = next->first;
		    unsigned int index = next->second;
		    queue.erase(next);

		    if(map->isFrontier(index))
		    {
				int block = map->convertToBlock(index);
				ROS_INFO("Checking the block #%d", block);
				if(!used[block] && moveTo(index) /* checkPath(start, index)*/ ) 
				{
					foundFrontier = true;
					//moveTo(index);
				}
				used[block] = true;
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

	    ROS_DEBUG("Checked %d cells.", cellCount);
	    delete [] plan;

		if(interrupted) 
			return 2;

	    if(foundFrontier) 
	    {
		    return 0;
	    }
	    else
	    {
		    if(cellCount > 20)
			    return 1;
		    else
			    return -1;
	    }
    }

    bool getMap() 
    {
	    if(!mGetMapClient_.isValid()) 
		    return false;

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
	init(argc, argv, "explore_service_node");

	ExploreService explore_service;

	spin();
	
	return 0;
}
