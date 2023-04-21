#pragma once

#include <rviz_rendering/objects/axes.hpp>
#include <ssl_vehicle_tracking_visualization/Shape.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <rviz_common/display.hpp>

namespace rviz_rendering
{
typedef std::shared_ptr<ssl_vehicle_tracking_visualization::Shape> ShapeSharedPtr;
typedef std::shared_ptr<rviz_rendering::Axes> AxesSharedPtr;
typedef std::shared_ptr<rviz_rendering::MovableText> MovableTextSharedPtr;
typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLineSharedPtr;
}  // namespace rviz_rendering

namespace ssl_vehicle_tracking_visualization
{

class Visual
{
private:
  Ogre::SceneNode* scene_node_ = nullptr; /**< Frame to move the frame of reference. **/

  rviz_common::DisplayContext* display_context_ = nullptr; /** Ogre manager for the display. **/
  Ogre::SceneManager* scene_manager_ = nullptr;            /** Ogre manager for the 3D scene. **/

  std::shared_ptr<rviz_rendering::Axes> axes_; /** Axes for indocation of the current pose. **/

  float axes_length_ = 0.0F;
  float axes_radius_ = 0.0F;
  float z_axes_shift_ = 0.0F;
  bool visible_ = false;

protected:
  void setPosition(const Ogre::Vector3& position);

  void setScale(const Ogre::Vector3& scale);

  void setOrientation(const Ogre::Quaternion& orientation);

  auto getPosition() -> const Ogre::Vector3&;

  auto getScale() -> const Ogre::Vector3&;

  auto getOrientation() -> const Ogre::Quaternion&;

  auto getSceneNode() -> Ogre::SceneNode*;

  auto getSceneManager() -> Ogre::SceneManager*;

  auto getDisplayContext() -> rviz_common::DisplayContext*;

public:
  /**
   * @brief Basic constructor for initialisation of an 3D object.
   * @details The scene manager is the global Ogre manager for the hole 3D scene.
   * The parent node is the node were all objects in this visualisation have to be
   * appended. The node parent, child will create a inheritance tree for frames.
   * @param scene_manager Global 3D scene manager.
   * @param parent_node Parent node of this object.
   * @param context Rviz display context
   */
  Visual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz_common::DisplayContext* display_context);

  /**
   * @brief Basic destructor to delete all children nodes.
   * @details Will use the ogre manager to delete frames.
   */
  virtual ~Visual();

  void setVisible(bool visible);

  void setZAxesShift(float z_axes_shift);

  void setAxesLength(float axes_length);

  void setAxesRadius(float axes_radius);

  auto isVisible() -> const bool;

  auto getZAxesShift() -> const float;

  auto getAxesLength() -> const float;

  auto getAxesRadius() -> const float;
};

}  // namespace ssl_vehicle_tracking_visualization
