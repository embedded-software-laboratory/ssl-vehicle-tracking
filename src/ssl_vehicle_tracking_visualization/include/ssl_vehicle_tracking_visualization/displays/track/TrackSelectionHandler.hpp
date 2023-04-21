#pragma once

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/interaction/selection_handler.hpp>

#include <ssl_vehicle_tracking_msgs/msg/track.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track_stamped.hpp>

#include "ssl_vehicle_tracking_visualization/AngleProperty.hpp"

namespace ssl_vehicle_tracking_visualization
{

class TrackSelectionHandler : public rviz_common::interaction::SelectionHandler
{
private:
  const ssl_vehicle_tracking_msgs::msg::Track* object_ = nullptr;
  rviz_common::properties::VectorProperty* position_property_ = nullptr;
  rviz_common::properties::AngleProperty* heading_angle_property_ = nullptr;
  rviz_common::properties::VectorProperty* velocity_property_ = nullptr;
  rviz_common::properties::AngleProperty* velocity_angle_property_ = nullptr;
  rviz_common::properties::VectorProperty* acceleration_property_ = nullptr;
  rviz_common::properties::VectorProperty* size_property_ = nullptr;
  rviz_common::properties::StringProperty* type_id_property_ = nullptr;

public:
  explicit TrackSelectionHandler(rviz_common::DisplayContext* context);

  Ogre::Vector3 getPosition();
  Ogre::Vector3 getVelocity();
  Ogre::Vector3 getAcceleration();
  Ogre::Vector3 getSize();
  float getHeadingAngle();
  float getVelocityAngle();
  QString getTypeIdString();
  QString getSourceIdString();
  ssl_vehicle_tracking_msgs::msg::Track::_track_id_type getInternalId();

  void createProperties(const rviz_common::interaction::Picked& obj,
                        rviz_common::properties::Property* parent_property) override;
  void updateProperties() override;

  const ssl_vehicle_tracking_msgs::msg::Track* getObject() const;
  void setObject(const ssl_vehicle_tracking_msgs::msg::Track* object);
};

}  // namespace ssl_vehicle_tracking_visualization
