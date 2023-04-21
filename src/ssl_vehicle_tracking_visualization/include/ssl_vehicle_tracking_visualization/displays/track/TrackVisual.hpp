#pragma once

#include <ssl_vehicle_tracking_msgs/msg/track.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track_stamped.hpp>

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <rviz_common/display.hpp>

#include "TrackSelectionHandler.hpp"

#include "ssl_vehicle_tracking_visualization/Visual.hpp"
#include "ssl_vehicle_tracking_visualization/LineObject.hpp"

namespace ssl_vehicle_tracking_visualization
{

class TrackVisual : public Visual
{
private:
  // Visuals
  ssl_vehicle_tracking_visualization::LineObjectSharedPtr track_indicator_ = nullptr;
  rviz_rendering::MovableTextSharedPtr type_name_text_ = nullptr;
  Ogre::SceneNode* text_node_ = nullptr;

  Ogre::Vector3 text_position_;

  // Handler
  std::shared_ptr<ssl_vehicle_tracking_visualization::TrackSelectionHandler> selection_handler_;

  // Parameter
  float line_width_;
  float line_height_;
  Ogre::ColourValue detected_color_;
  Ogre::ColourValue predicted_color_;

  bool calculateTransformation(const ssl_vehicle_tracking_msgs::msg::Track::ConstSharedPtr& message,
                               const std_msgs::msg::Header::ConstSharedPtr& header, Ogre::Vector3& pos,
                               Ogre::Quaternion& orient);

public:
  TrackVisual(Ogre::SceneNode* parent_node, rviz_common::DisplayContext* context);

  virtual ~TrackVisual();

  void updateMessage(const ssl_vehicle_tracking_msgs::msg::Track::ConstSharedPtr& observation,
                     const std_msgs::msg::Header::ConstSharedPtr& header);

  void setTextVisible(bool text_visible);

  void setTextColor(Ogre::ColourValue color);

  void setTextSize(float font_size);

  void setPredictedColor(Ogre::ColourValue color);

  void setDetectedColor(Ogre::ColourValue color);

  void setLineHeight(float line_height);

  void setLineWidth(float line_width);
};

}  // namespace ssl_vehicle_tracking_visualization
