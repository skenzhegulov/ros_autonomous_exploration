#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <autonomous_exploration/ExploreAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_explore");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	autonomous_exploration::ExploreGoal goal;
	autonomous_exploration::ExploreResultConstPtr feedback;
	bool success;
	char ch;

	actionlib::SimpleActionClient<autonomous_exploration::ExploreAction> ac("explore", true);
	
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	
	do {
		ac.sendGoal(goal);

		success = ac.waitForResult(ros::Duration(30.0));

		if(success) 
		{
			actionlib::SimpleClientGoalState state = ac.getState();
			feedback = ac.getResult();
			ROS_INFO("Action finished: %s", state.toString().c_str());
			
			visualization_msgs::Marker target_point;
			target_point.header.frame_id = "map";
			target_point.header.stamp = ros::Time::now();
			target_point.ns = "points";
			target_point.action = visualization_msgs::Marker::ADD;
			target_point.type = visualization_msgs::Marker::POINTS;
			target_point.pose.orientation.w = 1.0;
			target_point.scale.x = target_point.scale.y = 0.2;
			target_point.id = 0;

			target_point.color.r = 1.0;
			target_point.color.a = 1.0;
			
			target_point.points.push_back(feedback->target);

			marker_pub.publish(target_point);
		}
		else
		{
			ROS_INFO("Action did not finish before the time out.");
		}
		std::cin>>ch;
	} while(ch!='q');
	return 0;
}
