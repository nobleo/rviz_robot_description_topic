#include "robot_description_topic.h"

#include <rviz/display_context.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/robot/tf_link_updater.h>
#include <urdf/model.h>

namespace rviz_robot_description_topic
{
void linkUpdaterStatusFunction(
  rviz::StatusProperty::Level level, const std::string & link_name, const std::string & text,
  rviz::RobotModelDisplay * display)
{
  display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
}

RobotDescriptionTopicDisplay::RobotDescriptionTopicDisplay() : rviz::RobotModelDisplay()
{
  // Clear the parameter property
  robot_description_property_->~StringProperty();

  // Add a topic property
  description_topic_property_ = std::make_shared<rviz::RosTopicProperty>(
    "Description Topic", "/robot_description",
    QString::fromStdString(ros::message_traits::datatype<std_msgs::String>()),
    "robot_description topic to subscribe to.", this, SLOT(updateTopicProperty()));
}

void RobotDescriptionTopicDisplay::updateTopicProperty()
{
  unSubscribe();
  reset();
  subscribe();
}

void RobotDescriptionTopicDisplay::load()
{
  // Direct copy of the load function in RobotModelDisplay without the parameter reading
  clearStatuses();
  context_->queueRender();

  urdf::Model descr;
  if (!descr.initString(robot_description_)) {
    clear();
    setStatus(rviz::StatusProperty::Error, "URDF", "Failed to parse URDF model");
    return;
  }

  setStatus(rviz::StatusProperty::Ok, "URDF", "URDF parsed OK");
  robot_->load(descr);
  std::stringstream ss;
  for (const auto & name_link_pair : robot_->getLinks()) {
    const std::string & err = name_link_pair.second->getGeometryErrors();
    if (!err.empty()) ss << "\n• for link '" << name_link_pair.first << "':\n" << err;
  }
  if (ss.tellp())
    setStatus(
      rviz::StatusProperty::Error, "URDF",
      QString("Errors loading geometries:").append(ss.str().c_str()));

  robot_->update(rviz::TFLinkUpdater(
    context_->getFrameManager(), boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
    tf_prefix_property_->getStdString()));
}

void RobotDescriptionTopicDisplay::onEnable() { subscribe(); }

void RobotDescriptionTopicDisplay::onDisable()
{
  unSubscribe();
  reset();
}

void RobotDescriptionTopicDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  const auto topic_name = description_topic_property_->getTopicStd();
  ROS_INFO_STREAM("Subscribing to: " << topic_name);
  try {
    description_subscriber_ =
      update_nh_.subscribe(topic_name, 1, &RobotDescriptionTopicDisplay::descriptionCallback, this);
    setStatus(rviz::StatusProperty::Ok, "Description Topic", "OK");
  } catch (ros::Exception & e) {
    setStatus(
      rviz::StatusProperty::Error, "Description Topic", QString("Error subscribing: ") + e.what());
  }
}

void RobotDescriptionTopicDisplay::unSubscribe()
{
  if (description_subscriber_) {
    description_subscriber_.shutdown();
  }
}

void RobotDescriptionTopicDisplay::descriptionCallback(const std_msgs::String::ConstPtr msg)
{
  robot_description_ = msg->data;
  ROS_DEBUG("Received new robot_description via topic");
  load();
}

}  // namespace rviz_robot_description_topic

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_robot_description_topic::RobotDescriptionTopicDisplay, rviz::Display)
