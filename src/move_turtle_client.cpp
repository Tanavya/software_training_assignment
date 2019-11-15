#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <software_training_assignment/MoveTurtleAction.h>
#include <iostream>


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "test_move_turtle");

	actionlib::SimpleActionClient<software_training_assignment::MoveTurtleAction> ac("move_turtle", true);
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server started.");
	software_training_assignment::MoveTurtleGoal goal;
	//software_training_assignment::MoveTurtleActionGoal goal; //??

	float goal_x, goal_y;

	std::cout << "Enter x-coordinate: ";
	std::cin >> goal_x;
	std::cout << "Enter y-coordinate: ";
	std::cin >> goal_y;

	goal.goal_x = goal_x;
	goal.goal_y = goal_y;

	ac.sendGoal(goal);

	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	return 0;

}