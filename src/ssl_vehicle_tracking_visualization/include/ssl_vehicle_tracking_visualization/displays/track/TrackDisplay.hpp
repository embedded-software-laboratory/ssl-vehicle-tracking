#pragma once

#include <memory>

#include <boost/circular_buffer.hpp>
#include <map>

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <ssl_vehicle_tracking_msgs/msg/track_stamped.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track_array.hpp>

#include "ssl_vehicle_tracking_visualization/displays/track/TrackVisual.hpp"

namespace ssl_vehicle_tracking_visualization
{

/**
 * @authors Simon Schaefer
 * @date 18.11.2022
 * @brief Display to handel TrackArray in Rviz.
 * @details This will use a list of wheels to display the detections in Rviz.
 */
class TrackDisplay : public rviz_common::RosTopicDisplay<ssl_vehicle_tracking_msgs::msg::TrackArray>
{
  Q_OBJECT
private:
  std::map<ssl_vehicle_tracking_msgs::msg::Track::_track_id_type, float> times_elapsed_since_last_update_; /**< Pointer
                                                                                                   to actual visual
                                                                                                   objects. */
  std::map<ssl_vehicle_tracking_msgs::msg::Track::_track_id_type, std::shared_ptr<TrackVisual>> visuals_;  /**< Pointer
                                                                                                    to actual  visual
                                                                                                    objects. */
  std::map<ssl_vehicle_tracking_msgs::msg::Track::_track_id_type, ssl_vehicle_tracking_msgs::msg::Track::ConstSharedPtr>
      last_messages_received_; /**< Pointer to last message that was received. */
  std_msgs::msg::Header::ConstSharedPtr last_received_header_;

  rviz_common::properties::ColorProperty* detected_color_property_;  /**< Property to choose detected color. */
  rviz_common::properties::ColorProperty* predicted_color_property_; /**< Property to choose predicted color. */

  rviz_common::properties::FloatProperty* z_shift_property_property_; /**< Property to shift geometric objects. */
  rviz_common::properties::FloatProperty* line_width_property_;       /**< Property to choose the line width. */
  rviz_common::properties::FloatProperty* line_height_property_;      /**< Property to choose the line height. */
  rviz_common::properties::Property* visual_group_;

  rviz_common::properties::ColorProperty* text_color_property_;     /**< Property to choose text color. */
  rviz_common::properties::FloatProperty* text_font_size_property_; /**< Property to choose the text font size value. */
  rviz_common::properties::BoolProperty* text_group_;

  /**
   * @brief Callback function for incoming messages.
   * @details Will process all elements from an TrackArray and associate a visual for each element.
   * @param msg Incoming message.
   */
  void processMessage(ssl_vehicle_tracking_msgs::msg::TrackArray::ConstSharedPtr msg) override;

  void clear();

  void updateVisual(std::shared_ptr<TrackVisual> visual);

public:
  /**
   * @brief Basic constructor that creates the properties.
   * @details The properties are added automatically to rviz do to the inheritance to MessageFilterDisplay.
   */
  TrackDisplay();

  /**
   * @brief Basic destructor that frees the properties..
   */
  ~TrackDisplay() override;

public:
  /**
   * @brief Passes all properties to the visuals.
   * @details Will be called automatically by rviz.
   */
  void onInitialize() override;

  // Overrides from Display
  void update(float dt, float ros_dt) override;

  /**
   * @brief Clears the display.
   * @details Will be called automatically by rviz.
   */
  void reset() override;

protected:
  void onEnable() override;

  void onDisable() override;

private Q_SLOTS:

  void updateVisuals();
};

}  // namespace ssl_vehicle_tracking_visualization
