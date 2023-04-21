#include "ssl_vehicle_tracking_visualization/displays/track/TrackVisual.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ssl_vehicle_tracking_visualization
{

TrackVisual::TrackVisual(Ogre::SceneNode* parent_node, rviz_common::DisplayContext* context)
  : Visual(context->getSceneManager(), parent_node, context)
{
  // Create text frame
  this->text_node_ = parent_node->createChildSceneNode();
  this->text_node_->setVisible(false);
  this->type_name_text_ = std::make_shared<rviz_rendering::MovableText>("Test Test");
  // Attach text to scene
  this->type_name_text_->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
  this->text_node_->attachObject(type_name_text_.get());
  this->type_name_text_->showOnTop(true);

  // Create geometric objects
  this->track_indicator_ =
      std::make_shared<ssl_vehicle_tracking_visualization::LineObject>(this->getSceneManager(), this->getSceneNode());

  this->track_indicator_->setLineWidth(0.01f);

  // Make selectable
  this->selection_handler_ =
      rviz_common::interaction::createSelectionHandler<ssl_vehicle_tracking_visualization::TrackSelectionHandler>(
          this->getDisplayContext());
  this->selection_handler_->setObject(nullptr);
  this->selection_handler_->addTrackedObjects(this->getSceneNode());
}

TrackVisual::~TrackVisual()
{
  this->getSceneManager()->destroySceneNode(this->text_node_);
}

void TrackVisual::updateMessage(const ssl_vehicle_tracking_msgs::msg::Track::ConstSharedPtr& msg,
                                const std_msgs::msg::Header::ConstSharedPtr& header)
{
  // Hide scene of one of the inputs is null
  if (!msg || !header)
  {
    this->setVisible(false);
    return;
  }

  // Calculate the transformation based on the header
  Ogre::Vector3 pos;
  Ogre::Quaternion orient;

  if (!this->calculateTransformation(msg, header, pos, orient))
  {
    this->setVisible(false);
    return;
  }

  if (this->track_indicator_->isEmpty())
    this->text_position_ = Ogre::Vector3(msg->states.back().x, msg->states.back().y, 0.0f);

  this->track_indicator_->addPoint(Ogre::Vector3(msg->states.back().x, msg->states.back().y, 0.0f),
                                   msg->states.back().state == "matched" ? this->detected_color_ :
                                                                           this->predicted_color_);

  // Update text
  this->type_name_text_->setCaption(msg->track_id);
  this->text_node_->setPosition(pos + text_position_ + Ogre::Vector3(0.0F, 0.0F, 0.0F + this->getZAxesShift()));

  // Update visual
  this->setPosition(pos);
  this->setOrientation(orient);

  // Show scene if it was hidden before
  this->setVisible(true);

  // Push object to selection handler
  this->selection_handler_->setObject(&*msg);
}

void TrackVisual::setTextColor(Ogre::ColourValue color)
{
  this->type_name_text_->setColor(color);
}

void TrackVisual::setTextVisible(bool text_visible)
{
  this->text_node_->setVisible(text_visible);
}

void TrackVisual::setTextSize(float font_size)
{
  this->type_name_text_->setCharacterHeight(font_size);
}

bool TrackVisual::calculateTransformation(const ssl_vehicle_tracking_msgs::msg::Track::ConstSharedPtr& message,
                                          const std_msgs::msg::Header::ConstSharedPtr& header, Ogre::Vector3& pos,
                                          Ogre::Quaternion& orient)
{
  auto orientation = tf2::Quaternion();
  orientation.setRPY(0, 0, 0);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.orientation = tf2::toMsg(orientation);

  if (!this->getDisplayContext()->getFrameManager()->transform(header->frame_id, header->stamp, pose, pos, orient))
  {
    std::string error;
    this->getDisplayContext()->getFrameManager()->transformHasProblems(header->frame_id, header->stamp, error);
    RVIZ_COMMON_LOG_DEBUG("Unable to transform object message: " + error);
    return false;
  }

  return true;
}

void TrackVisual::setPredictedColor(Ogre::ColourValue color)
{
  this->predicted_color_ = color;
}

void TrackVisual::setDetectedColor(Ogre::ColourValue color)
{
  this->detected_color_ = color;
}

void TrackVisual::setLineHeight(float line_height)
{
  this->line_height_ = line_height;
  this->track_indicator_->setLineHeight(this->line_height_);
}

void TrackVisual::setLineWidth(float line_width)
{
  this->line_width_ = line_width;
  this->track_indicator_->setLineWidth(this->line_width_);
}

}  // namespace ssl_vehicle_tracking_visualization
