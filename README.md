# rviz_robot_description_topic

This plugin allows you to display a robot model in rviz using a topic instead of a parameter.

## Changing models

The `robot_model` plugin in rviz allows you to display a robot model using a parameter.
This is useful if you have a static robot model that you want to display.
However, if you want to display a robot model that is changing, you need to use a topic.
This plugin allows you to display a robot model using a topic.

## ROS 2 preparation

In ROS 2 the robot model is already published as a message.
If you have a ROS1 code base but are preparing for a ROS2 migration, you can already convert the `/robot_description` parameter to a topic.
This allows for less refactoring during the ROS 2 migration.

If you plan on using the [ros1_bridge](https://github.com/ros2/ros1_bridge) to connect ROS 1 and ROS 2, a robot description topic will pass this bridge just fine, whereas a parameter will not.
