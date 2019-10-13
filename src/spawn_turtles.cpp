#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <cstdlib>

using namespace std;

void spawnTurtle(float x, float y, float theta, string name) {

	ros::NodeHandle nh;
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	srv.request.x = x;	
	srv.request.y = y;
	srv.request.theta = theta;
	srv.request.name = name;
	spawnTurtle.call(srv);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "spawn_turtles");

	ros::NodeHandle nh;

	spawnTurtle(5, 5, 0, "stationary_turtle");
	spawnTurtle(25, 10, 0, "moving_turtle");

}