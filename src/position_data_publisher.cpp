#include "ros/ros.h"
#include "std_msgs/String.h"
#include <turtlesim/Pose.h>
#include <software_training_assignment/PositionData.h>
#include <sstream>
#include <cmath>

float moving_turtle_x, moving_turtle_y, distance_x = -1, distance_y = -1, distance;

void setMovingTurtlePose(const turtlesim::Pose::ConstPtr& msg) {
    moving_turtle_x = msg->x;
    moving_turtle_y = msg->y;
}

void calculateDistance(const turtlesim::Pose::ConstPtr& msg) {
    distance_x = abs(moving_turtle_x - (msg->x));
    distance_y = abs(moving_turtle_y - (msg->y));
    distance = sqrt(pow(distance_x, 2) + pow(distance_y, 2));
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "position_data_publisher");
  ros::NodeHandle n;

  ros::Publisher position_pub = n.advertise<software_training_assignment::PositionData>("position_data", 1000);

  ros::Rate loop_rate(10);

  
  //ros::Subscriber sub1 = n.subscribe("/moving_turtle/pose", 1000, setMovingTurtlePose);
  //ros::Subscriber sub2 = n.subscribe("/stationary_turtle/pose", 1000, calculateDistance);

  //ros::spin();

  ros::Subscriber sub1 = n.subscribe("/moving_turtle/pose", 1000, setMovingTurtlePose);
  ros::Subscriber sub2 = n.subscribe("/stationary_turtle/pose", 1000, calculateDistance);
  

  while (ros::ok()) {
    

    software_training_assignment::PositionData msg;
    msg.distance_x = distance_x;
    msg.distance_y = distance_y;
    msg.distance = distance;

    ROS_INFO("distance_x: [%f]", msg.distance_x);
    ROS_INFO("distance_y: [%f]", msg.distance_y);
    ROS_INFO("distance  : [%f]", msg.distance);

    position_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }
}