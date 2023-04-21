#pragma once

#include <memory>

#include <boost/circular_buffer.hpp>
#include <map>

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <ssl_vehicle_tracking_msgs/msg/wheel_stamped.hpp>
#include <ssl_vehicle_tracking_msgs/msg/wheel.hpp>
#include <ssl_vehicle_tracking_msgs/msg/wheel_array.hpp>

#include "ssl_vehicle_tracking_visualization/displays/wheel/WheelVisual.hpp"

namespace ssl_vehicle_tracking_visualization
{

/**
 * @authors Simon Schaefer
 * @date 18.11.2022
 * @brief Display to handel WheelArray in Rviz.
 * @details This will use a list of wheels to display the detections in Rviz.
 */
class WheelDisplay : public rviz_common::RosTopicDisplay<ssl_vehicle_tracking_msgs::msg::WheelArray>
{
  Q_OBJECT
private:
  std::map<ssl_vehicle_tracking_msgs::msg::Wheel::_wheel_id_type, float> times_elapsed_since_last_update_; /**< Pointer
                                                                                                   to actual visual
                                                                                                   objects. */
  std::map<ssl_vehicle_tracking_msgs::msg::Wheel::_wheel_id_type, std::shared_ptr<WheelVisual>> visuals_;  /**< Pointer
                                                                                                    to actual  visual
                                                                                                    objects. */
  std::map<ssl_vehicle_tracking_msgs::msg::Wheel::_wheel_id_type, ssl_vehicle_tracking_msgs::msg::Wheel::ConstSharedPtr>
      last_messages_received_; /**< Pointer to last message that was received. */
  std_msgs::msg::Header::ConstSharedPtr last_received_header_;

  rviz_common::properties::ColorProperty* max_color_property_ = nullptr; /**< Property to choose maximum color. */
  rviz_common::properties::FloatProperty* max_value_property_ = nullptr; /**< Property yo choose the maximum value. */

  rviz_common::properties::ColorProperty* min_color_property_ = nullptr; /**< Property to choose minium color. */
  rviz_common::properties::FloatProperty* min_value_property_ = nullptr; /**< Property yo choose the minumum value. */

  rviz_common::properties::FloatProperty* z_shift_property_property_ =
      nullptr; /**< Property to shift geometric objects. */
  rviz_common::properties::FloatProperty* wireframe_radius_property_ =
      nullptr; /**< Property to choose the wireframe radius value. */
  rviz_common::properties::FloatProperty* axes_radius_property_ =
      nullptr; /**< Property to choose the axes radius value. */
  rviz_common::properties::FloatProperty* axes_length_property_ =
      nullptr; /**< Property to choose the axes length value. */

  rviz_common::properties::ColorProperty* text_color_property_ = nullptr; /**< Property to choose text color. */
  rviz_common::properties::FloatProperty* text_font_size_property_ =
      nullptr; /**< Property to choose the text font size value. */
  rviz_common::properties::BoolProperty* text_group_ = nullptr;

  /**
   * @brief Callback function for incoming messages.
   * @details Will process all elements from an WheelArray and associate a visual for each element.
   * @param msg Incoming message.
   */
  void processMessage(ssl_vehicle_tracking_msgs::msg::WheelArray::ConstSharedPtr msg) override;

public:
  /**
   * @brief Basic constructor that creates the properties.
   * @details The properties are added automatically to rviz do to the inheritance to MessageFilterDisplay.
   */
  WheelDisplay();

  /**
   * @brief Basic destructor that frees the properties..
   */
  ~WheelDisplay() override;

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
