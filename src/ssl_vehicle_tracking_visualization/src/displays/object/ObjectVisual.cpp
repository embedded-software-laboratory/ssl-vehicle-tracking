#include "ssl_vehicle_tracking_visualization/displays/object/ObjectVisual.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ssl_vehicle_tracking_visualization
{

ObjectVisual::ObjectVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                           rviz_common::DisplayContext* context)
  : Visual(scene_manager, parent_node, context)
{
  // Create text frame
  this->text_node_ = parent_node->createChildSceneNode();
  this->text_node_->setVisible(false);

  this->type_name_text_ = std::make_shared<rviz_rendering::MovableText>("Test Test");

  // Attach text to scene
  this->type_name_text_->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
  this->type_name_text_->showOnTop(true);
  this->text_node_->attachObject(type_name_text_.get());
  this->text_node_->setPosition(Ogre::Vector3(-0.150F / 2.0F, 107.0F / 2.0F, 0.0F));

  // Create body objects
  this->body_lat_ = std::make_shared<ssl_vehicle_tracking_visualization::Shape>(
      ssl_vehicle_tracking_visualization::Shape::Type::Cube, this->getSceneManager(), this->getSceneNode());
  this->body_long_ = std::make_shared<ssl_vehicle_tracking_visualization::Shape>(
      ssl_vehicle_tracking_visualization::Shape::Type::Cube, this->getSceneManager(), this->getSceneNode());

  this->body_long_->setPosition(Ogre::Vector3(0.150F / 2.0F, 0.0F, 0.0F));
  this->body_lat_->setPosition(Ogre::Vector3(0.150F / 2.0F, 0.0F, 0.0F));

  this->body_lat_->setOrientation(Ogre::Quaternion(Ogre::Degree(0), Ogre::Vector3::UNIT_Z));
  this->body_long_->setOrientation(Ogre::Quaternion(Ogre::Degree(0), Ogre::Vector3::UNIT_Z));

  // Create wheel indicators
  this->front_left_wheel_indicator_ = std::make_shared<ssl_vehicle_tracking_visualization::Shape>(
      ssl_vehicle_tracking_visualization::Shape::Type::Cylinder, this->getSceneManager(), this->getSceneNode());
  this->front_right_wheel_indicator_ = std::make_shared<ssl_vehicle_tracking_visualization::Shape>(
      ssl_vehicle_tracking_visualization::Shape::Type::Cylinder, this->getSceneManager(), this->getSceneNode());
  this->rear_left_wheel_indicator_ = std::make_shared<ssl_vehicle_tracking_visualization::Shape>(
      ssl_vehicle_tracking_visualization::Shape::Type::Cylinder, this->getSceneManager(), this->getSceneNode());
  this->rear_right_wheel_indicator_ = std::make_shared<ssl_vehicle_tracking_visualization::Shape>(
      ssl_vehicle_tracking_visualization::Shape::Type::Cylinder, this->getSceneManager(), this->getSceneNode());

  this->front_left_wheel_indicator_->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
  this->front_right_wheel_indicator_->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
  this->rear_left_wheel_indicator_->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
  this->rear_right_wheel_indicator_->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));

  // Make selectable
  this->selection_handler_ =
      rviz_common::interaction::createSelectionHandler<ssl_vehicle_tracking_visualization::ObjectSelectionHandler>(
          this->getDisplayContext());
  this->selection_handler_->setObject(nullptr);
  this->selection_handler_->addTrackedObjects(this->getSceneNode());
}

ObjectVisual::~ObjectVisual()
{
  this->getSceneManager()->destroySceneNode(this->text_node_);
}

void ObjectVisual::updateVisual(const ssl_vehicle_tracking_msgs::msg::Object::ConstSharedPtr& msg,
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

  const Ogre::ColourValue GREEN = Ogre::ColourValue(0.24F, 0.86F, 0.59F, 1.0F);
  const Ogre::ColourValue RED = Ogre::ColourValue(1.0F, 0.29F, 0.36F, 1.0F);

  this->front_right_wheel_indicator_->setColor(msg->wheel_detected[0] ? GREEN : RED);
  this->front_left_wheel_indicator_->setColor(msg->wheel_detected[1] ? GREEN : RED);
  this->rear_right_wheel_indicator_->setColor(msg->wheel_detected[2] ? GREEN : RED);
  this->rear_left_wheel_indicator_->setColor(msg->wheel_detected[3] ? GREEN : RED);

  // Update text
  this->type_name_text_->setCaption(std::to_string(msg->object_id));
  this->text_node_->setPosition(pos + Ogre::Vector3(0.0F, 0.0F, 0.0f + this->getZAxesShift()));

  // Update visual
  this->setPosition(pos);
  this->setOrientation(orient);

  // Show scene if it was hidden before
  this->setVisible(true);

  // Push object to selection handler
  this->selection_handler_->setObject(&*msg);
}

void ObjectVisual::setObjectColor(Ogre::ColourValue color)
{
  this->front_right_wheel_indicator_->setColor(color);
  this->front_left_wheel_indicator_->setColor(color);
  this->rear_right_wheel_indicator_->setColor(color);
  this->rear_left_wheel_indicator_->setColor(color);

  this->body_long_->setColor(color);
  this->body_lat_->setColor(color);
}

void ObjectVisual::setTextColor(Ogre::ColourValue color)
{
  this->type_name_text_->setColor(color);
}

void ObjectVisual::setTextVisible(bool text_visible)
{
  this->text_node_->setVisible(text_visible);
}

void ObjectVisual::setObjectCornerRadius(float radius)
{
  this->body_long_->setScale(Ogre::Vector3(0.150, 0.107f - 2.0f * radius, radius));
  this->body_lat_->setScale(Ogre::Vector3(0.150 - 2.0f * radius, 0.107f, radius));

  auto inidactor_size = Ogre::Vector3(radius * 2.0F, radius * 0.98F, radius * 2.0F);
  this->front_left_wheel_indicator_->setScale(inidactor_size);
  this->front_right_wheel_indicator_->setScale(inidactor_size);
  this->rear_left_wheel_indicator_->setScale(inidactor_size);
  this->rear_right_wheel_indicator_->setScale(inidactor_size);

  this->front_left_wheel_indicator_->setPosition(Ogre::Vector3(0.150F - radius, 0.107F / 2.0F - radius, 0.0F));
  this->front_right_wheel_indicator_->setPosition(Ogre::Vector3(0.150F - radius, -0.107F / 2.0F + radius, 0.0F));
  this->rear_left_wheel_indicator_->setPosition(Ogre::Vector3(0.0F + radius, 0.107F / 2.0F - radius, 0.0F));
  this->rear_right_wheel_indicator_->setPosition(Ogre::Vector3(0.0F + radius, -0.107F / 2.0F + radius, 0.0F));
}

void ObjectVisual::setTextSize(float font_size)
{
  this->type_name_text_->setCharacterHeight(font_size);
}

bool ObjectVisual::calculateTransformation(const ssl_vehicle_tracking_msgs::msg::Object::ConstSharedPtr& message,
                                           const std_msgs::msg::Header::ConstSharedPtr& header, Ogre::Vector3& pos,
                                           Ogre::Quaternion& orient)
{
  auto orientation = tf2::Quaternion();
  orientation.setRPY(0, 0, message->pose.theta);

  geometry_msgs::msg::Pose pose;
  pose.position.x = message->pose.x;
  pose.position.y = message->pose.y;
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

}  // namespace ssl_vehicle_tracking_visualization
