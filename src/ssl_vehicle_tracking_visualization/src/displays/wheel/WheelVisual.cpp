#include "ssl_vehicle_tracking_visualization/displays/wheel/WheelVisual.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ssl_vehicle_tracking_visualization
{

WheelVisual::WheelVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                         rviz_common::DisplayContext* context)
  : Visual(scene_manager, parent_node, context)
{
  // Create geometric objects
  this->wheel_indicator_ = std::make_shared<ssl_vehicle_tracking_visualization::Shape>(
      ssl_vehicle_tracking_visualization::Shape::Type::Cube, this->getSceneManager(), this->getSceneNode());

  // Create text frame
  this->text_node_ = parent_node->createChildSceneNode();
  this->text_node_->setVisible(false);
  this->type_name_text_ = std::make_shared<rviz_rendering::MovableText>("Test Test");

  // Attach text to scene
  this->type_name_text_->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
  this->text_node_->attachObject(type_name_text_.get());
  this->type_name_text_->showOnTop(true);

  // Make selectable
  this->selection_handler_ =
      rviz_common::interaction::createSelectionHandler<ssl_vehicle_tracking_visualization::WheelSelectionHandler>(
          this->getDisplayContext());
  this->selection_handler_->setObject(nullptr);
  this->selection_handler_->addTrackedObjects(this->getSceneNode());
}

WheelVisual::~WheelVisual()
{
  this->getSceneManager()->destroySceneNode(this->text_node_);
}

void WheelVisual::updateVisual(const ssl_vehicle_tracking_msgs::msg::Wheel::ConstSharedPtr& msg,
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

  float interpolation_factor = std::max(this->range_.first, std::min(this->range_.second, msg->intensity));
  auto color =
      (1.0f - interpolation_factor) * this->color_range_.first + interpolation_factor * this->color_range_.second;

  this->wheel_indicator_->setColor(color);

  auto inidactor_size = Ogre::Vector3(this->wireframe_radius_ * 2.0F, this->wireframe_radius_ * 2.0F,
                                      interpolation_factor * this->wireframe_radius_ * 4.0F);
  this->wheel_indicator_->setScale(inidactor_size);

  this->wheel_indicator_->setPosition(Ogre::Vector3(0.0f, 0.0f, interpolation_factor * this->wireframe_radius_ * 2.0F));

  // Update text
  this->type_name_text_->setCaption(std::to_string(msg->wheel_id));
  this->text_node_->setPosition(pos + Ogre::Vector3(0.0F, 0.0F, 0.0F + this->getZAxesShift()));

  // Update visual
  this->setPosition(pos);
  this->setOrientation(orient);

  // Show scene if it was hidden before
  this->setVisible(true);

  // Push object to selection handler
  this->selection_handler_->setObject(&*msg);
}

void WheelVisual::setObjectColor(Ogre::ColourValue color)
{
  return;
}

void WheelVisual::setTextColor(Ogre::ColourValue color)
{
  this->type_name_text_->setColor(color);
}

void WheelVisual::setTextVisible(bool text_visible)
{
  this->text_node_->setVisible(text_visible);
}

void WheelVisual::setRadius(float radius)
{
  this->wireframe_radius_ = radius;

  auto inidactor_size =
      Ogre::Vector3(this->wireframe_radius_ * 2.0F, this->wireframe_radius_ * 2.0F, this->wireframe_radius_ * 4.0F);
  this->wheel_indicator_->setScale(inidactor_size);
}

void WheelVisual::setFontSize(float font_size)
{
  this->type_name_text_->setCharacterHeight(font_size);
}

bool WheelVisual::calculateTransformation(const ssl_vehicle_tracking_msgs::msg::Wheel::ConstSharedPtr& message,
                                          const std_msgs::msg::Header::ConstSharedPtr& header, Ogre::Vector3& pos,
                                          Ogre::Quaternion& orient)
{
  auto orientation = tf2::Quaternion();
  orientation.setRPY(0, 0, 0);

  geometry_msgs::msg::Pose pose;
  pose.position.x = message->x;
  pose.position.y = message->y;
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

void WheelVisual::setRange(float min_value, float max_value)
{
  this->range_.first = min_value;
  this->range_.second = max_value;
}

void WheelVisual::setColorRange(Ogre::ColourValue min_color, Ogre::ColourValue max_color)
{
  this->color_range_.first = min_color;
  this->color_range_.second = max_color;
}

}  // namespace ssl_vehicle_tracking_visualization
