#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <software_training_assignment/MoveTurtleAction.h>
#include <software_training_assignment/PositionData.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

using namespace std;

typedef actionlib::SimpleActionServer<software_training_assignment::MoveTurtleAction> Server;

float moving_turtle_x, moving_turtle_y;

void setMovingTurtlePose(const turtlesim::Pose::ConstPtr& msg) {
	moving_turtle_x = msg->x;
	moving_turtle_y = msg->y;
}


class MoveTurtleAction {

	protected:
		ros::NodeHandle nh;
		Server as;
		std::string action_name;

		software_training_assignment::MoveTurtleFeedback feedback;
		software_training_assignment::MoveTurtleResult result;

		//float moving_turtle_x, moving_turtle_y;

	public:

		/*
		void setMovingTurtlePose(const turtlesim::Pose::ConstPtr& msg) {
    		moving_turtle_x = msg->x;
    		moving_turtle_y = msg->y;
		}*/

		MoveTurtleAction(string name) : as(nh, name, boost::bind(&MoveTurtleAction::executeCB, this, _1), false), action_name(name) {
			as.start();
		}

		~MoveTurtleAction(void) {}




		void quick_move(const software_training_assignment::MoveTurtleGoalConstPtr &goal) {

			ros::Subscriber pose_sub = nh.subscribe("/moving_turtle/pose", 1000, setMovingTurtlePose);
  			ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("moving_turtle/cmd_vel", 1000);
    		
    		ROS_INFO("%s: Executing, moving moving_turtle to (%f %f).", action_name.c_str(), goal->goal_x, goal->goal_y);

			bool success = true;

			ros::spinOnce();

			ROS_INFO("Current x: %f", moving_turtle_x);
			ROS_INFO("Current y: %f", moving_turtle_y);

			float relative_angle = atan(((goal->goal_y - moving_turtle_y))/(goal->goal_x - moving_turtle_x));
			float angular_speed = relative_angle;
			float target_distance = sqrt(pow(moving_turtle_x - goal->goal_x, 2) + pow(moving_turtle_y - goal->goal_y, 2)); 

			ROS_INFO("Relative Angle  : %f", relative_angle);
			ROS_INFO("Angular Speed   : %f", angular_speed);
			ROS_INFO("Target Distance : %f", target_distance);

			geometry_msgs::Twist msg;

			msg.linear.x = msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = 0;
			msg.angular.z = angular_speed;

			float current_angle = 0;
			float current_distance = 0;

			ros::Rate r(1);

			ros::Time begin = ros::Time::now();
			r.sleep();
			cmd_vel_pub.publish(msg);

			
			msg.angular.z = 0;
			msg.linear.x = target_distance;
			r.sleep();

			cmd_vel_pub.publish(msg);


			msg.angular.z = -angular_speed;
			msg.linear.x = 0;
			r.sleep();
			cmd_vel_pub.publish(msg);

			r.sleep();
			ros::spinOnce();

			ROS_INFO("Final x: %f", moving_turtle_x);
			ROS_INFO("Final y: %f", moving_turtle_y);

			if (success) {
				result.time_taken = (ros::Time::now() - begin).toSec();
				ROS_INFO("%s: Succeeded", action_name.c_str());
				as.setSucceeded(result);
			}

		}

		void executeCB(const software_training_assignment::MoveTurtleGoalConstPtr &goal) {
			
			//quick_move(goal);
			//return;

			ros::Subscriber pose_sub = nh.subscribe("/moving_turtle/pose", 1000, setMovingTurtlePose);
  			ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("moving_turtle/cmd_vel", 1000);
    		
    		ROS_INFO("%s: Executing, moving moving_turtle to (%f %f).", action_name.c_str(), goal->goal_x, goal->goal_y);

			bool success = true;

			ros::spinOnce();

			float relative_angle = atan(((goal->goal_y - moving_turtle_y))/(goal->goal_x - moving_turtle_x));
			float angular_speed = 0.1;
			float target_distance = sqrt(pow(moving_turtle_x - goal->goal_x, 2) + pow(moving_turtle_y - goal->goal_y, 2)); 
			float linear_speed = 0.5;

			ROS_INFO("Relative Angle  : %f", relative_angle);
			ROS_INFO("Angular Speed   : %f", angular_speed);
			ROS_INFO("Target Distance : %f", target_distance);

			geometry_msgs::Twist msg;

			msg.linear.x = msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = 0;
			msg.angular.z = angular_speed;


			float current_angle = 0;
			float current_distance = 0;

			ros::Rate r(100);

			ros::Time begin = ros::Time::now();
			r.sleep();
			cmd_vel_pub.publish(msg);
			
			
			while (current_angle < relative_angle) {
				ROS_INFO("Current Angle: %f", current_angle);
				cmd_vel_pub.publish(msg);
				
				float change = (ros::Time::now() - begin).toSec();
				current_angle = angular_speed * change;
				
				if (as.isPreemptRequested() || !ros::ok()) {
			        ROS_INFO("%s: Preempted", action_name.c_str());
			        as.setPreempted();
			        success = false;
			        break;
			    }

				feedback.distance_to_goal = target_distance;
				as.publishFeedback(feedback);
				r.sleep();
			}

			msg.linear.x = linear_speed;
			msg.angular.z = 0;
			result.time_taken = (ros::Time::now() - begin).toSec();

			begin = ros::Time::now();

			while (current_distance < target_distance) {
				ROS_INFO("Current Distance: %f", current_distance);
				cmd_vel_pub.publish(msg);
				float change = (ros::Time::now() - begin).toSec();
				current_distance = linear_speed * change;
				feedback.distance_to_goal = target_distance - current_distance;
				as.publishFeedback(feedback);				
				r.sleep();
			}

		
			result.time_taken += (ros::Time::now() - begin).toSec();

			if (success) {
				result.time_taken = (ros::Time::now() - begin).toSec();
				ROS_INFO("%s: Succeeded", action_name.c_str());
				//set action state to succeeded
				as.setSucceeded(result);
			}
		}

};

int main(int argc, char** argv){

  ros::init(argc, argv, "move_turtle");

  MoveTurtleAction move_turtle("move_turtle");

  ros::spin();

  return 0;
}
