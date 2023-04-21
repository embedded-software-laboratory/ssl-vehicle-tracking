#ifndef SSL_VEHICLE_TRACKING_VISUALIZATION__OBJECTS__LINE_HPP_
#define SSL_VEHICLE_TRACKING_VISUALIZATION__OBJECTS__LINE_HPP_

#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

#include <optional>

#include "rviz_rendering/objects/object.hpp"
#include "ssl_vehicle_tracking_visualization/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
class Quaternion;
class Any;
class ColourValue;
}  // namespace Ogre

namespace ssl_vehicle_tracking_visualization
{

/* Represents a straight wireframe line between two points. */
class LineObject : public rviz_rendering::Object
{
protected:
  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr manual_object_material_;

  float line_width_;
  float line_height_;
  std::optional<Ogre::Vector3> last_point_;
  std::optional<Ogre::ColourValue> last_color_;
  std::optional<Ogre::Vector3> last_shift_;

  bool empty_ = true;

public:
  /**
   * \brief Constructor
   * @param manager Scene manager this object is a part of
   * @param parent_node A scene node to use as the parent of this object. Uses the root scene
   * node if null.
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC explicit LineObject(Ogre::SceneManager* manager,
                                                                Ogre::SceneNode* parent_node);

  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  virtual ~LineObject();

  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void clear();

  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void estimatedSegmentCount(int buffer_length);

  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void addPoint(Ogre::Vector3 point, Ogre::ColourValue color = Ogre::ColourValue::Green);

  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void setVisible(bool visible);

  /**
   * \brief Set the position of this object
   * @param Position vector position to set to.
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void setPosition(const Ogre::Vector3& position) override;

  /**
   * \brief Set the orientation of the object
   * @param Orientation quaternion orientation to set to.
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void setOrientation(const Ogre::Quaternion& orientation) override;

  /**
   * \brief Set the scale of the object.  Always relative to the identity orientation of the object.
   * @param Scale vector scale to set to.
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void setScale(const Ogre::Vector3& scale) override;

  /**
   * \brief Set the width of the line.
   *
   * @param line_width : new line width.
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void setLineWidth(const float line_width);

  /**
   * \brief Set the height of the line.
   *
   * @param line_height : new line height.
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void setLineHeight(const float line_height);

  /**
   * \brief Get the local position of this object
   * @return The position
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  const Ogre::Vector3& getPosition() override;

  /**
   * \brief Get the local orientation of this object
   * @return The orientation
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  const Ogre::Quaternion& getOrientation() override;

  /**
   * \brief Get the line width of this object
   * @return The line width
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  const float& getLineWidth();

  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  const bool& isEmpty();

  /**
   * \brief Get the line height of this object
   * @return The line height
   */
  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  const float& getLineHeight();

  SSL_VEHICLE_TRACKING_VISUALIZATION_PUBLIC
  void setUserData(const Ogre::Any& data) override;

  SSL_VEHICLE_TRACKING_VISUALIZATION_LOCAL
  void setColor(float r, float g, float b, float a) override;
};

typedef std::shared_ptr<ssl_vehicle_tracking_visualization::LineObject> LineObjectSharedPtr;

}  // namespace ssl_vehicle_tracking_visualization

#endif  // SSL_VEHICLE_TRACKING_VISUALIZATION__OBJECTS__LINE_HPP_