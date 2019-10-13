#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportAbsolute.h>
#include <software_training_assignment/ResetMovingTurtle.h>
#include <cstdlib>

using namespace std;

bool reset_turtle(software_training_assignment::ResetMovingTurtle::Request &req, software_training_assignment::ResetMovingTurtle::Response &res) {
	ros::NodeHandle nh;
	ros::ServiceClient resetTurtle = nh.serviceClient<turtlesim::TeleportAbsolute>("/moving_turtle/teleport_absolute");
	turtlesim::TeleportAbsolute srv;
	srv.request.x = 0;
	srv.request.y = 0;
	srv.request.theta = 0;
	res.success = resetTurtle.call(srv);
	return true;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "reset_moving_turtle");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("reset_moving_turtle", reset_turtle);
	ROS_INFO("Ready to reset_moving_turtle.");
	ros::spin();

}