#pragma once

#include <ssl_vehicle_tracking_msgs/msg/wheel.hpp>
#include <ssl_vehicle_tracking_msgs/msg/wheel_stamped.hpp>

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/axes.hpp>

#include <rviz_common/display.hpp>

#include "WheelSelectionHandler.hpp"

#include "ssl_vehicle_tracking_visualization/Visual.hpp"

namespace ssl_vehicle_tracking_visualization
{

class WheelVisual : public Visual
{
private:
  // Visuals
  rviz_rendering::ShapeSharedPtr wheel_indicator_ = nullptr;
  rviz_rendering::MovableTextSharedPtr type_name_text_ = nullptr;
  Ogre::SceneNode* text_node_ = nullptr;

  // Handler
  std::shared_ptr<ssl_vehicle_tracking_visualization::WheelSelectionHandler> selection_handler_;

  // Parameter
  float wireframe_radius_;
  std::pair<float, float> range_;
  std::pair<Ogre::ColourValue, Ogre::ColourValue> color_range_;

  bool calculateTransformation(const ssl_vehicle_tracking_msgs::msg::Wheel::ConstSharedPtr& message,
                               const std_msgs::msg::Header::ConstSharedPtr& header, Ogre::Vector3& pos,
                               Ogre::Quaternion& orient);

public:
  WheelVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz_common::DisplayContext* context);

  virtual ~WheelVisual();

  void updateVisual(const ssl_vehicle_tracking_msgs::msg::Wheel::ConstSharedPtr& observation,
                    const std_msgs::msg::Header::ConstSharedPtr& header);

  void setTextVisible(bool text_visible);

  void setObjectColor(Ogre::ColourValue color);

  void setRadius(float radius);

  void setTextColor(Ogre::ColourValue color);

  void setFontSize(float font_size);

  void setRange(float min_value, float max_value);

  void setColorRange(Ogre::ColourValue min_color, Ogre::ColourValue max_color);
};

}  // namespace ssl_vehicle_tracking_visualization
