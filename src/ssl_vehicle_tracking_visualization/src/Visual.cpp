#include "ssl_vehicle_tracking_visualization/Visual.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>

namespace ssl_vehicle_tracking_visualization
{

Visual::Visual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
               rviz_common::DisplayContext* display_context)
  : display_context_(display_context)
  , scene_manager_(scene_manager)
  , scene_node_(parent_node->createChildSceneNode())
  , axes_(std::make_shared<rviz_rendering::Axes>(scene_manager, scene_node_))
{
  // Default initialisation
  this->setVisible(this->visible_);
  this->setAxesLength(this->axes_length_);
  this->setAxesRadius(this->axes_radius_);
  this->setOrientation(Ogre::Quaternion(Ogre::Degree(0), Ogre::Vector3::UNIT_Z));
  this->setPosition(Ogre::Vector3(0.0F, 0.0F, 0.0F));
  this->setScale(Ogre::Vector3(1.0F, 1.0F, 1.0F));
}

Visual::~Visual()
{
  this->scene_manager_->destroySceneNode(scene_node_);
}

void Visual::setVisible(bool visible)
{
  this->visible_ = visible;
  this->scene_node_->setVisible(visible);
}

void Visual::setZAxesShift(float z_axes_shift)
{
  this->z_axes_shift_ = z_axes_shift;
}

void Visual::setAxesLength(float axes_length)
{
  this->axes_length_ = axes_length;
  this->axes_->set(this->axes_length_, this->axes_radius_);
}

void Visual::setAxesRadius(float axes_radius)
{
  this->axes_radius_ = axes_radius;
  this->axes_->set(this->axes_length_, this->axes_radius_);
}

void Visual::setPosition(const Ogre::Vector3& position)
{
  auto pos = position;
  pos.z = pos.z + this->z_axes_shift_;

  this->scene_node_->setPosition(pos);
}

void Visual::setScale(const Ogre::Vector3& scale)
{
  this->scene_node_->setScale(scale);
}

void Visual::setOrientation(const Ogre::Quaternion& orientation)
{
  this->scene_node_->setOrientation(orientation);
}

auto Visual::isVisible() -> const bool
{
  return this->visible_;
}

auto Visual::getZAxesShift() -> const float
{
  return this->z_axes_shift_;
}

auto Visual::getAxesLength() -> const float
{
  return this->axes_length_;
}

auto Visual::getAxesRadius() -> const float
{
  return this->axes_radius_;
}

auto Visual::getPosition() -> const Ogre::Vector3&
{
  return this->scene_node_->getPosition();
}

auto Visual::getScale() -> const Ogre::Vector3&
{
  return this->scene_node_->getScale();
}

auto Visual::getOrientation() -> const Ogre::Quaternion&
{
  return this->scene_node_->getOrientation();
}

auto Visual::getSceneNode() -> Ogre::SceneNode*
{
  return this->scene_node_;
}

auto Visual::getSceneManager() -> Ogre::SceneManager*
{
  return this->scene_manager_;
}

auto Visual::getDisplayContext() -> rviz_common::DisplayContext*
{
  return this->display_context_;
}

}  // namespace ssl_vehicle_tracking_visualization
