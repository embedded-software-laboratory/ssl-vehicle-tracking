#include "ssl_vehicle_tracking_visualization/displays/wheel/WheelVisual.hpp"
#include "ssl_vehicle_tracking_visualization/displays/wheel/WheelDisplay.hpp"

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

void WheelDisplay::processMessage(ssl_vehicle_tracking_msgs::msg::WheelArray::ConstSharedPtr msg)
{
  for (auto observation : msg->wheels)
  {
    last_messages_received_[observation.wheel_id] =
        std::make_shared<ssl_vehicle_tracking_msgs::msg::Wheel>(observation);
    times_elapsed_since_last_update_[observation.wheel_id] = 0;
    // RVIZ_COMMON_LOG_INFO("Received vehicle: " + std::to_string(observation.observation.vehicle_id));
  }
  last_received_header_ = std::make_shared<std_msgs::msg::Header>(msg->header);

  std::vector<ssl_vehicle_tracking_msgs::msg::Wheel::_wheel_id_type> index_to_remove;
  for (auto visual : visuals_)
  {
    index_to_remove.push_back(visual.first);
  }
  for (auto index : index_to_remove)
  {
    times_elapsed_since_last_update_.erase(index);
    visuals_.erase(index);
    last_messages_received_.erase(index);
  }
}

WheelDisplay::WheelDisplay()
{
  auto visuals_group = new Property("Visuals", "", "Contains all visual related properties.", this);

  // Create properties

  min_color_property_ =
      new ColorProperty("Min Color", QColor(204, 7, 30), "Color to draw indicator with minimum pressure.",
                        visuals_group, SLOT(updateVisuals()), this);
  max_color_property_ =
      new ColorProperty("Max Color", QColor(204, 7, 30), "Color to draw indicator with maximum pressure.",
                        visuals_group, SLOT(updateVisuals()), this);

  min_value_property_ = new FloatProperty("Min Value", 0.0f, "Will set the lowest ecpected pressure value.",
                                          visuals_group, SLOT(updateVisuals()), this);
  max_value_property_ = new FloatProperty("Max Value", 1.0f, "Will set the highest ecpected pressure value.",
                                          visuals_group, SLOT(updateVisuals()), this);

  wireframe_radius_property_ = new FloatProperty("Radius", 0.01, "Sets the radius for the geometry corner.",
                                                 visuals_group, SLOT(updateVisuals()), this);
  wireframe_radius_property_->setMin(0);
  wireframe_radius_property_->setMax(1);

  axes_radius_property_ = new FloatProperty("Axes Radius", 0.01, "Sets the radius for the origin axes.", visuals_group,
                                            SLOT(updateVisuals()), this);
  axes_radius_property_->setMin(0);
  axes_radius_property_->setMax(1);

  axes_length_property_ = new FloatProperty("Axes Length", 0.05, "Sets the length for the origin axes.", visuals_group,
                                            SLOT(updateVisuals()), this);
  axes_length_property_->setMin(0);
  axes_length_property_->setMax(1);

  z_shift_property_property_ = new FloatProperty("Z shift.", 0.00f, "Will shift the geometry alogn the z-axis.",
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

WheelDisplay::~WheelDisplay()
{
  delete min_color_property_;
  delete max_color_property_;
  delete min_value_property_;
  delete max_value_property_;
  delete text_color_property_;
  delete text_font_size_property_;
  delete z_shift_property_property_;
  delete wireframe_radius_property_;
  delete axes_radius_property_;
  delete axes_length_property_;
  delete text_group_;
}

void WheelDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  updateVisuals();
}

void WheelDisplay::update(float dt, float ros_dt)
{
  RosTopicDisplay::update(dt, ros_dt);

  for (auto msg_pair : last_messages_received_)
  {
    // RVIZ_COMMON_LOG_INFO("Pre-Update vehicle: " + std::to_string(msg_pair.second->observation.vehicle_id));

    int index = msg_pair.second->wheel_id;
    // Check if the visual was already created
    if (visuals_.find(index) == visuals_.end())
    {
      // Add new visual
      visuals_[index] = std::make_shared<WheelVisual>(context_->getSceneManager(), scene_node_, context_);
      visuals_[index]->setTextColor(text_color_property_->getOgreColor());
      visuals_[index]->setTextVisible(text_group_->getBool());
      visuals_[index]->setFontSize(text_font_size_property_->getFloat());
      visuals_[index]->setZAxesShift(z_shift_property_property_->getFloat());
      visuals_[index]->setAxesLength(axes_length_property_->getFloat());
      visuals_[index]->setAxesRadius(axes_radius_property_->getFloat());
      visuals_[index]->setRadius(wireframe_radius_property_->getFloat());
      visuals_[index]->setRange(min_value_property_->getFloat(), max_value_property_->getFloat());
      visuals_[index]->setColorRange(min_color_property_->getOgreColor(), max_color_property_->getOgreColor());
    }

    //  Update already existing visuals
    visuals_[index]->updateVisual(msg_pair.second, this->last_received_header_);
    // RVIZ_COMMON_LOG_INFO("Update vehicle: " + std::to_string(index));
  }

  // RVIZ_COMMON_LOG_INFO("Update dt: " + std::to_string(dt) + " ros_dt: " + std::to_string(ros_dt));
  std::vector<ssl_vehicle_tracking_msgs::msg::Wheel::_wheel_id_type> index_to_remove;
  for (auto visual : visuals_)
  {
    if (times_elapsed_since_last_update_[visual.first] > 1)
    {
      visual.second->updateVisual(nullptr, nullptr);
      times_elapsed_since_last_update_[visual.first] = 0;
      index_to_remove.push_back(visual.first);
    }
    times_elapsed_since_last_update_[visual.first] += ros_dt * 1e-9;
  }
  for (auto index : index_to_remove)
  {
    times_elapsed_since_last_update_.erase(index);
    visuals_.erase(index);
    last_messages_received_.erase(index);
    // RVIZ_COMMON_LOG_INFO("Reset vehicle: " + std::to_string(index));
  }
}

void WheelDisplay::reset()
{
  RosTopicDisplay::reset();
  for (auto& visual : visuals_)
  {
    if (visual.second)
      visual.second->updateVisual(nullptr, nullptr);
  }
  this->visuals_.clear();
  this->last_messages_received_.clear();
  this->last_received_header_ = nullptr;
}

void WheelDisplay::onEnable()
{
  RosTopicDisplay::onEnable();
}

void WheelDisplay::onDisable()
{
  RosTopicDisplay::onDisable();
  for (auto& visual : visuals_)
  {
    if (visual.second)
      visual.second->updateVisual(nullptr, nullptr);
  }
  this->visuals_.clear();
  this->last_messages_received_.clear();
  this->last_received_header_ = nullptr;
}

void WheelDisplay::updateVisuals()
{
  Ogre::ColourValue max_color = max_color_property_->getOgreColor();
  Ogre::ColourValue min_color = min_color_property_->getOgreColor();
  Ogre::ColourValue text_color = text_color_property_->getOgreColor();

  for (auto& visual : visuals_)
  {
    if (!visual.second)
      continue;

    visual.second->setRange(min_value_property_->getFloat(), max_value_property_->getFloat());
    visual.second->setTextVisible(text_group_->getBool());
    visual.second->setFontSize(text_font_size_property_->getFloat());
    visual.second->setAxesRadius(axes_radius_property_->getFloat());
    visual.second->setAxesLength(axes_length_property_->getFloat());
    visual.second->setRadius(wireframe_radius_property_->getFloat());
    visual.second->setZAxesShift(this->z_shift_property_property_->getFloat());
    visual.second->setColorRange(min_color, max_color);
    visual.second->setTextColor(text_color);
  }
}

}  // namespace ssl_vehicle_tracking_visualization

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ssl_vehicle_tracking_visualization::WheelDisplay, rviz_common::Display)
