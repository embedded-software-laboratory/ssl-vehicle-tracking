#pragma once

#include <ssl_vehicle_tracking_msgs/msg/object.hpp>
#include <ssl_vehicle_tracking_msgs/msg/object_stamped.hpp>

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/axes.hpp>

#include <rviz_common/display.hpp>

#include "ObjectSelectionHandler.hpp"

#include "ssl_vehicle_tracking_visualization/Visual.hpp"

namespace ssl_vehicle_tracking_visualization
{

class ObjectVisual : public Visual
{
private:
  rviz_rendering::ShapeSharedPtr front_line_ = nullptr;
  rviz_rendering::ShapeSharedPtr left_line_ = nullptr;
  rviz_rendering::ShapeSharedPtr right_line_ = nullptr;
  rviz_rendering::ShapeSharedPtr rear_line_ = nullptr;
  rviz_rendering::ShapeSharedPtr front_left_wheel_indicator_ = nullptr;
  rviz_rendering::ShapeSharedPtr front_right_wheel_indicator_ = nullptr;
  rviz_rendering::ShapeSharedPtr rear_left_wheel_indicator_ = nullptr;
  rviz_rendering::ShapeSharedPtr rear_right_wheel_indicator_ = nullptr;
  rviz_rendering::MovableTextSharedPtr type_name_text_ = nullptr;

  rviz_rendering::ShapeSharedPtr body_long_ = nullptr;
  rviz_rendering::ShapeSharedPtr body_lat_ = nullptr;

  std::shared_ptr<ssl_vehicle_tracking_visualization::ObjectSelectionHandler> selection_handler_;

  Ogre::SceneNode* text_node_ = nullptr;

  bool calculateTransformation(const ssl_vehicle_tracking_msgs::msg::Object::ConstSharedPtr& message,
                               const std_msgs::msg::Header::ConstSharedPtr& header, Ogre::Vector3& pos,
                               Ogre::Quaternion& orient);

public:
  ObjectVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz_common::DisplayContext* context);

  virtual ~ObjectVisual();

  void updateVisual(const ssl_vehicle_tracking_msgs::msg::Object::ConstSharedPtr& observation,
                    const std_msgs::msg::Header::ConstSharedPtr& header);


  void setObjectColor(Ogre::ColourValue color);

  void setObjectCornerRadius(float radius);

  void setTextVisible(bool text_visible);

  void setTextColor(Ogre::ColourValue color);

  void setTextSize(float font_size);
};

}  // namespace ssl_vehicle_tracking_visualization
