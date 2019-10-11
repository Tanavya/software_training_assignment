Create a ros package called software_training_assignment
Install the turtlesim package(hint: you shouldn't be downloading any source code from github. You should be using a package manager).
Create a launch file that launches the turtlesim node with a background colour of (r=128, b=0, g=128) 
Hint: http://wiki.ros.org/Names and http://wiki.ros.org/roslaunch/XML/group are useful. You need to make sure the parameter values are set in the correct ROS namepace (bugged upstream)
Write a node that:
Clears any existing turtles
Spawns a turtle named "stationary_turtle" at x = 5, y = 5
Spawns a second turtle named "moving_turtle" at x = 25, y = 10
Create a service that resets the "moving_turtle" to its starting position. The service response should be whether or not it was successful.
Create a publisher that publishes a custom msg. This custom msgs should have 3 integer fields that correspond with the x and y distances of "stationary_turtle" to "moving turtle", as well as the distance between the two turtles.
Create an action that moves "moving_turtle" to a waypoint in a straight line by publishing geometry_msgs/Twist msgs to turtleX/cmd_vel.The action's goal is an absolute position of the waypoint, feedback is distance to the goal, and result is the time it took to reach the goal. You should define a custom action file. To easily test your action, you can use the rqt_action rqt plugin. 
Add your node you created to your launch file. Make sure your node doesn't crash if the turtlesim_node has not come up yet.
Create a .gitignore file so that only your source files and CMakelists are committed
Push your package to a personal github repo and msg Melvin with the link(root directory of the repo should only contain your package folder).
