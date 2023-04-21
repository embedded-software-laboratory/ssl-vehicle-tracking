#include "ssl_vehicle_tracking_visualization/displays/wheel/WheelSelectionHandler.hpp"

#include <QtGui/QtGui>

namespace ssl_vehicle_tracking_visualization
{

WheelSelectionHandler::WheelSelectionHandler(rviz_common::DisplayContext* context)
  : rviz_common::interaction::SelectionHandler(context)
{
}

Ogre::Vector3 WheelSelectionHandler::getPosition()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { static_cast<float>(object_->x), static_cast<float>(object_->y), 0 };
}

Ogre::Vector3 WheelSelectionHandler::getVelocity()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { 0.0f, 0.0f, 0.0f };
}

Ogre::Vector3 WheelSelectionHandler::getAcceleration()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { 0.0f, 0.0f, 0.0f };
}

Ogre::Vector3 WheelSelectionHandler::getSize()
{
  if (object_ == nullptr)
    return { 0.0f, 0.0f, 0.0f };
  return { 0.22f, 0.107f, 0.07f };
}

float WheelSelectionHandler::getHeadingAngle()
{
  return 0.0f;
}

float WheelSelectionHandler::getVelocityAngle()
{
  if (object_ == nullptr)
    return 0.0f;
  return 0.0f;
}

QString WheelSelectionHandler::getTypeIdString()
{
  if (object_ == nullptr)
    return QString::fromStdString("Unknown");
  return QString::fromStdString("µCar");
}

QString WheelSelectionHandler::getSourceIdString()
{
  if (object_ == nullptr)
    return QString::fromStdString("Unknown");
  return QString::fromStdString("IPS");
}

uint16_t WheelSelectionHandler::getInternalId()
{
  if (object_ == nullptr)
    return 0;
  return object_->wheel_id;
}

void WheelSelectionHandler::createProperties(const rviz_common::interaction::Picked& obj,
                                             rviz_common::properties::Property* parent_property)
{
  SelectionHandler::createProperties(obj, parent_property);

  uint16_t internal_id = getInternalId();
  QString object_caption = "Wheel " + getSourceIdString() + " " + QString::number(internal_id);

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

void WheelSelectionHandler::updateProperties()
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

const ssl_vehicle_tracking_msgs::msg::Wheel* WheelSelectionHandler::getObject() const
{
  return object_;
}

void WheelSelectionHandler::setObject(const ssl_vehicle_tracking_msgs::msg::Wheel* object)
{
  object_ = object;
}

}  // namespace ssl_vehicle_tracking_visualization
