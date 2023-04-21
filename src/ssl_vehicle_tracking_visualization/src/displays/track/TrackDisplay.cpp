#include "ssl_vehicle_tracking_visualization/displays/track/TrackVisual.hpp"
#include "ssl_vehicle_tracking_visualization/displays/track/TrackDisplay.hpp"

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

void TrackDisplay::processMessage(ssl_vehicle_tracking_msgs::msg::TrackArray::ConstSharedPtr msg)
{
  for (auto observation : msg->tracks)
  {
    last_messages_received_[observation.track_id] =
        std::make_shared<ssl_vehicle_tracking_msgs::msg::Track>(observation);
    times_elapsed_since_last_update_[observation.track_id] = 0;
  }
  last_received_header_ = std::make_shared<std_msgs::msg::Header>(msg->header);
}

void TrackDisplay::clear()
{
  for (auto& visual : visuals_)
  {
    if (!visual.second)
      continue;
    visual.second->updateMessage(nullptr, nullptr);
  }
  this->visuals_.clear();
  this->last_messages_received_.clear();
  this->last_received_header_ = nullptr;
}

void TrackDisplay::updateVisual(std::shared_ptr<TrackVisual> visual)
{
  if (!visual)
    return;

  float z_axes_shift = this->z_shift_property_property_->getFloat();
  bool show_text = this->text_group_->getBool();
  float font_size = this->text_font_size_property_->getFloat();
  float line_width = this->line_width_property_->getFloat();
  float line_height = this->line_height_property_->getFloat();

  visual->setPredictedColor(this->predicted_color_property_->getOgreColor());
  visual->setDetectedColor(this->detected_color_property_->getOgreColor());
  visual->setZAxesShift(z_axes_shift);
  visual->setLineWidth(line_width);
  visual->setLineHeight(line_height);

  visual->setTextColor(this->text_color_property_->getOgreColor());
  visual->setTextVisible(show_text);
  visual->setTextSize(font_size);
}

TrackDisplay::TrackDisplay()
{
  // Create visual properties
  this->visual_group_ = new Property("Visuals", "", "Contains all visual related properties.", this);

  this->detected_color_property_ = new ColorProperty("Detected Color", QColor(37, 110, 255),
                                                     "Color to draw line if an wheel was "
                                                     "associated.",
                                                     this->visual_group_, SLOT(updateVisuals()), this);
  this->predicted_color_property_ = new ColorProperty("Predicted Color", QColor(37, 110, 255),
                                                      "Color to draw the line if a gab was closed using a predictor.",
                                                      this->visual_group_, SLOT(updateVisuals()), this);

  this->line_width_property_ = new FloatProperty("Width", 0.005, "Sets the width of the track line.",
                                                 this->visual_group_, SLOT(updateVisuals()), this);
  this->line_width_property_->setMin(0);

  this->line_height_property_ = new FloatProperty("Height", 0.003, "Sets the height of the track line.",
                                                  this->visual_group_, SLOT(updateVisuals()), this);
  this->line_height_property_->setMin(0);

  this->z_shift_property_property_ = new FloatProperty("Z shift.", 0.0015f, "Will shift the geometry alogn the z-axis.",
                                                       this->visual_group_, SLOT(updateVisuals()), this);

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

TrackDisplay::~TrackDisplay()
{
  delete this->line_height_property_;
  delete this->line_width_property_;
  delete this->detected_color_property_;
  delete this->predicted_color_property_;
  delete this->text_color_property_;
  delete this->text_font_size_property_;
  delete this->z_shift_property_property_;
  delete this->text_group_;
  delete this->visual_group_;
}

void TrackDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  updateVisuals();
}

void TrackDisplay::update(float dt, float ros_dt)
{
  RosTopicDisplay::update(dt, ros_dt);

  for (auto msg_pair : last_messages_received_)
  {
    auto index = msg_pair.second->track_id;
    // Check if the visual was already created if not add new one
    if (visuals_.find(index) == visuals_.end())
    {
      // Add new visual
      auto visual = std::make_shared<TrackVisual>(scene_node_, context_);
      this->updateVisual(visual);

      visuals_[index] = visual;
    }

    //  Update existing visuals
    visuals_[index]->updateMessage(msg_pair.second, this->last_received_header_);
  }

  std::list<ssl_vehicle_tracking_msgs::msg::Track::_track_id_type> index_to_remove;
  for (auto visual : visuals_)
  {
    if (times_elapsed_since_last_update_[visual.first] > 1)
    {
      visual.second->updateMessage(nullptr, nullptr);
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
  }
}

void TrackDisplay::reset()
{
  RosTopicDisplay::reset();
  this->clear();
  for (auto& visual : visuals_)
  {
    if (!visual.second)
      continue;
    visual.second->updateMessage(nullptr, nullptr);
  }
  this->visuals_.clear();
  this->last_messages_received_.clear();
  this->last_received_header_ = nullptr;
}

void TrackDisplay::onEnable()
{
  RosTopicDisplay::onEnable();
}

void TrackDisplay::onDisable()
{
  RosTopicDisplay::onDisable();
  for (auto& visual : visuals_)
  {
    if (!visual.second)
      continue;
    visual.second->updateMessage(nullptr, nullptr);
  }
  this->visuals_.clear();
  this->last_messages_received_.clear();
  this->last_received_header_ = nullptr;
}

void TrackDisplay::updateVisuals()
{
  for (auto& visual_pair : visuals_)
    this->updateVisual(visual_pair.second);
}

}  // namespace ssl_vehicle_tracking_visualization

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ssl_vehicle_tracking_visualization::TrackDisplay, rviz_common::Display)
