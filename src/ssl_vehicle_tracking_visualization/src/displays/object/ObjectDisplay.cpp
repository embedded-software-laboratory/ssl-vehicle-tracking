#include "ssl_vehicle_tracking_visualization/displays/object/ObjectVisual.hpp"
#include "ssl_vehicle_tracking_visualization/displays/object/ObjectDisplay.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>

#include <std_msgs/msg/header.hpp>

using namespace rviz_common::properties;

namespace ssl_vehicle_tracking_visualization
{

void ObjectDisplay::processMessage(ssl_vehicle_tracking_msgs::msg::ObjectArray::ConstSharedPtr msg)
{
  for (auto observation : msg->objects)
  {
    last_messages_received_[observation.object_id] =
        std::make_shared<ssl_vehicle_tracking_msgs::msg::Object>(observation);
    times_elapsed_since_last_update_[observation.object_id] = 0;
  }
  last_received_header_ = std::make_shared<std_msgs::msg::Header>(msg->header);
}

ObjectDisplay::ObjectDisplay()
{
  auto visuals_group = new Property("Geometry", "", "Contains all geometry related properties.", this);

  geometric_color_property_ = new ColorProperty("Color", QColor(252, 252, 252), "Color to draw all geometry.",
                                                visuals_group, SLOT(updateVisuals()), this);

  wireframe_radius_property_ = new FloatProperty("Corner Radius", 0.01, "Sets the radius for the geometry corner.",
                                                 visuals_group, SLOT(updateVisuals()), this);
  wireframe_radius_property_->setMin(0);
  wireframe_radius_property_->setMax(1);

  axes_radius_property_ = new FloatProperty("Axes Radius", 0.01, "Sets the radius for the origin axes.", visuals_group,
                                            SLOT(updateAxes()), this);
  axes_radius_property_->setMin(0);
  axes_radius_property_->setMax(1);

  axes_length_property_ = new FloatProperty("Axes Length", 0.05, "Sets the length for the origin axes.", visuals_group,
                                            SLOT(updateAxes()), this);
  axes_length_property_->setMin(0);
  axes_length_property_->setMax(1);

  z_shift_property_property_ = new FloatProperty("Z shift.", 0.01f, "Will shift the geometry alogn the z-axis.",
                                                 visuals_group, SLOT(updateVisuals()), this);

  // Create text properties
  this->text_group_ =
      new BoolProperty("Text", "", "Contains all text related properties.", this, SLOT(updateVisuals()), this);
  this->text_group_->setDisableChildrenIfFalse(true);
  this->text_color_property_ = new ColorProperty("Color", QColor(94, 94, 94), "Color to draw texts.", this->text_group_,
                                                 SLOT(updateVisuals()), this);

  this->text_font_size_property_ =
      new FloatProperty("Size", 0.05, "Size of the text shown.", text_group_, SLOT(updateVisuals()), this);
  this->text_font_size_property_->setMin(0);
  this->text_font_size_property_->setMax(1);
}

ObjectDisplay::~ObjectDisplay()
{
  delete geometric_color_property_;
  delete text_color_property_;
  delete text_font_size_property_;
  delete z_shift_property_property_;
  delete wireframe_radius_property_;
  delete axes_radius_property_;
  delete axes_length_property_;
  delete text_group_;
}

void ObjectDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  updateVisuals();
}

void ObjectDisplay::update(float dt, float ros_dt)
{
  RosTopicDisplay::update(dt, ros_dt);

  for (auto msg_pair : last_messages_received_)
  {
    // RVIZ_COMMON_LOG_INFO("Pre-Update vehicle: " + std::to_string(msg_pair.second->observation.vehicle_id));

    int index = msg_pair.second->object_id;
    // Check if the visual was already created
    if (visuals_.find(index) == visuals_.end())
    {
      // Add new visual
      visuals_[index] = std::make_shared<ObjectVisual>(context_->getSceneManager(), scene_node_, context_);
      visuals_[index]->setObjectColor(geometric_color_property_->getOgreColor());
      visuals_[index]->setTextColor(text_color_property_->getOgreColor());
      visuals_[index]->setTextVisible(text_group_->getBool());
      visuals_[index]->setTextSize(text_font_size_property_->getFloat());
      visuals_[index]->setZAxesShift(z_shift_property_property_->getFloat());
      visuals_[index]->setAxesLength(axes_length_property_->getFloat());
      visuals_[index]->setAxesRadius(axes_radius_property_->getFloat());
      visuals_[index]->setObjectCornerRadius(wireframe_radius_property_->getFloat());
    }

    //  Update already existing visuals
    visuals_[index]->updateVisual(msg_pair.second, this->last_received_header_);
  }

  this->removeOutdatedVisuals(ros_dt);
}

void ObjectDisplay::reset()
{
  RosTopicDisplay::reset();
  this->resetVisuals();
}

void ObjectDisplay::onEnable()
{
  RosTopicDisplay::onEnable();
}

void ObjectDisplay::onDisable()
{
  RosTopicDisplay::onDisable();
  this->resetVisuals();
}

void ObjectDisplay::updateVisuals()
{
  Ogre::ColourValue geometric_color = geometric_color_property_->getOgreColor();
  Ogre::ColourValue text_color = text_color_property_->getOgreColor();
  for (auto& visual : visuals_)
  {
    if (!visual.second)
      continue;
    visual.second->setTextVisible(text_group_->getBool());
    visual.second->setTextSize(text_font_size_property_->getFloat());
    visual.second->setAxesRadius(axes_radius_property_->getFloat());
    visual.second->setAxesLength(axes_length_property_->getFloat());
    visual.second->setObjectCornerRadius(wireframe_radius_property_->getFloat());
    visual.second->setZAxesShift(this->z_shift_property_property_->getFloat());

    visual.second->setObjectColor(geometric_color);
    visual.second->setTextColor(text_color);
  }
}
void ObjectDisplay::resetVisuals()
{
  for (auto& visual : visuals_)
  {
    if (!visual.second)
      continue;
    visual.second->updateVisual(nullptr, nullptr);
  }
  this->visuals_.clear();
  this->last_messages_received_.clear();
  this->last_received_header_ = nullptr;
}

void ObjectDisplay::removeOutdatedVisuals(float nanoseconds_elapsed)
{
  std::vector<ssl_vehicle_tracking_msgs::msg::Object::_object_id_type> index_to_remove;
  for (auto visual : visuals_)
  {
    times_elapsed_since_last_update_[visual.first] += nanoseconds_elapsed * 1e-9;
    if (times_elapsed_since_last_update_[visual.first] <= 1)
      continue;

    visual.second->updateVisual(nullptr, nullptr);
    times_elapsed_since_last_update_[visual.first] = 0;
    index_to_remove.push_back(visual.first);
  }
  for (auto index : index_to_remove)
  {
    times_elapsed_since_last_update_.erase(index);
    visuals_.erase(index);
    times_elapsed_since_last_update_.erase(index);
    last_messages_received_.erase(index);
  }
}

}  // namespace ssl_vehicle_tracking_visualization

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ssl_vehicle_tracking_visualization::ObjectDisplay, rviz_common::Display)
