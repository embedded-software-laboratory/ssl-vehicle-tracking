#include "ssl_vehicle_tracking_visualization/displays/track/TrackSelectionHandler.hpp"

#include <QtGui/QtGui>

namespace ssl_vehicle_tracking_visualization
{

TrackSelectionHandler::TrackSelectionHandler(rviz_common::DisplayContext* context)
  : rviz_common::interaction::SelectionHandler(context)
{
}

Ogre::Vector3 TrackSelectionHandler::getPosition()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { 0.0f, 0.0f, 0.0f };
  // return { static_cast<float>(object_->x), static_cast<float>(object_->y), 0 };
}

Ogre::Vector3 TrackSelectionHandler::getVelocity()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { 0.0f, 0.0f, 0.0f };
}

Ogre::Vector3 TrackSelectionHandler::getAcceleration()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { 0.0f, 0.0f, 0.0f };
}

Ogre::Vector3 TrackSelectionHandler::getSize()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { 0.22f, 0.107f, 0.07f };
}

float TrackSelectionHandler::getHeadingAngle()
{
  return 0.0f;
}

float TrackSelectionHandler::getVelocityAngle()
{
  if (object_ == nullptr)
    return 0.0f;
  return 0.0f;
}

QString TrackSelectionHandler::getTypeIdString()
{
  if (object_ == nullptr)
    return QString::fromStdString("Unknown");
  return QString::fromStdString("µCar");
}

QString TrackSelectionHandler::getSourceIdString()
{
  if (object_ == nullptr)
    return QString::fromStdString("Unknown");
  return QString::fromStdString("IPS");
}

ssl_vehicle_tracking_msgs::msg::Track::_track_id_type TrackSelectionHandler::getInternalId()
{
  if (object_ == nullptr)
    return 0;
  return object_->track_id;
}

void TrackSelectionHandler::createProperties(const rviz_common::interaction::Picked& obj,
                                             rviz_common::properties::Property* parent_property)
{
  SelectionHandler::createProperties(obj, parent_property);

  ssl_vehicle_tracking_msgs::msg::Track::_track_id_type internal_id = getInternalId();
  QString object_caption = "Track " + getSourceIdString() + " " + QString::fromStdString(internal_id);

  auto* object_group = new rviz_common::properties::Property(object_caption, QVariant(), "", parent_property);
  properties_.push_back(object_group);

  position_property_ = new rviz_common::properties::VectorProperty("Position", getPosition(), "", object_group);
  position_property_->setReadOnly(true);

  heading_angle_property_ =
      new rviz_common::properties::AngleProperty("Heading angle", getHeadingAngle(), "", object_group);
  heading_angle_property_->setReadOnly(true);

  velocity_angle_property_ =
      new rviz_common::properties::AngleProperty("Velocity angle", getVelocityAngle(), "", object_group);
  velocity_angle_property_->setReadOnly(true);

  velocity_property_ = new rviz_common::properties::VectorProperty("Velocity", getVelocity(), "", object_group);
  velocity_property_->setReadOnly(true);

  acceleration_property_ =
      new rviz_common::properties::VectorProperty("Acceleration", getAcceleration(), "", object_group);
  acceleration_property_->setReadOnly(true);

  type_id_property_ = new rviz_common::properties::StringProperty("Type Id", getTypeIdString(), "", object_group);
  type_id_property_->setReadOnly(true);

  size_property_ = new rviz_common::properties::VectorProperty("Size", getSize(), "", object_group);
  size_property_->setReadOnly(true);

  //  object_group->expand();
}

void TrackSelectionHandler::updateProperties()
{
  SelectionHandler::updateProperties();
  position_property_->setVector(getPosition());
  heading_angle_property_->setFloat(getHeadingAngle());
  velocity_property_->setVector(getVelocity());
  velocity_angle_property_->setFloat(getVelocityAngle());
  acceleration_property_->setVector(getAcceleration());
  type_id_property_->setString(getTypeIdString());
  size_property_->setVector(getSize());
}

const ssl_vehicle_tracking_msgs::msg::Track* TrackSelectionHandler::getObject() const
{
  return object_;
}

void TrackSelectionHandler::setObject(const ssl_vehicle_tracking_msgs::msg::Track* object)
{
  object_ = object;
}

}  // namespace ssl_vehicle_tracking_visualization
