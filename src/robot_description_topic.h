#pragma once

#include <rviz/default_plugin/robot_model_display.h>
#include <std_msgs/String.h>

namespace rviz
{
class RosTopicProperty;
}  // namespace rviz

namespace rviz_robot_description_topic
{
// Inherit from RobotModelDisplay to get all the default functionality.
class RobotDescriptionTopicDisplay : public rviz::RobotModelDisplay
{
  Q_OBJECT
public:
  RobotDescriptionTopicDisplay();
  virtual ~RobotDescriptionTopicDisplay() = default;

private Q_SLOTS:
  void updateTopicProperty();

protected:
  /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models. */
  virtual void load();

  void onEnable() override;
  void onDisable() override;

  void subscribe();
  void unSubscribe();
  void descriptionCallback(const std_msgs::String::ConstPtr msg);

  std::shared_ptr<rviz::RosTopicProperty> description_topic_property_;

  ros::Subscriber description_subscriber_;
};

}  // namespace rviz_robot_description_topic
