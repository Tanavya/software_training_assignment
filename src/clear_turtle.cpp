#include "ros/ros.h"
#include <turtlesim/Kill.h>
#include <std_srvs/Empty.h>
#include <cstdlib>


int main(int argc, char **argv) {

  ros::init(argc, argv, "clear_turtle");

  ros::NodeHandle nh;

  ros::ServiceClient resetClient = nh.serviceClient<std_srvs::Empty>("/reset");
  std_srvs::Empty srv1;
  resetClient.call(srv1);

  ros::ServiceClient killClient = nh.serviceClient<turtlesim::Kill>("/kill");
  turtlesim::Kill srv2;
  srv2.request.name = "turtle1";
  killClient.call(srv2);

}