#include "ssl_vehicle_tracking_visualization/LineObject.hpp"

#include <string>
#include <iostream>
#include <utility>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreResourceGroupManager.h>

#include <rviz_rendering/material_manager.hpp>

namespace ssl_vehicle_tracking_visualization
{

LineObject::LineObject(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node) : Object(manager)
{
  static uint32_t count = 0;
  std::string entity_name = "MyLine" + std::to_string(count++);

  this->manual_object_ = manager->createManualObject();
  this->scene_node_ = parent_node->createChildSceneNode();
  this->scene_node_->attachObject(manual_object_);

  // this->manual_object_material_ =
  // Ogre::MaterialManager::getSingleton().getByName("ssl_vehicle_tracking_visualization/Trajectory");
  // this->manual_object_material_->load();

  std::string material_name_ = entity_name + "Material";
  this->manual_object_material_ = rviz_rendering::MaterialManager::createMaterialWithShadowsAndLighting(material_name_);

  this->clear();
}

LineObject::~LineObject()
{
  if (scene_node_->getParentSceneNode())
  {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
  }
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);
  manual_object_material_->unload();
}

void LineObject::clear()
{
  this->manual_object_->clear();
  this->last_point_ = {};
  this->last_color_ = {};
}

const bool& LineObject::isEmpty()
{
  return this->empty_;
}

void LineObject::estimatedSegmentCount(int segment_count)
{
  manual_object_->estimateVertexCount((segment_count)*10);
}

void LineObject::addPoint(Ogre::Vector3 point, Ogre::ColourValue color)
{
  this->empty_ = false;

  if (!this->last_point_)
  {
    this->last_point_ = point;
    this->last_color_ = color;
    return;
  }

  Ogre::Vector3 direction_vector = point - *this->last_point_;
  if (direction_vector.length() < 0.01)
    return;
  direction_vector.normalise();
  Ogre::Quaternion rotation(Ogre::Math::ATan2(direction_vector.y, direction_vector.x), Ogre::Vector3::UNIT_Z);
  Ogre::Vector3 shift(0, line_width_, 0);
  shift = rotation * shift;

  Ogre::Vector3 z_shift(0, 0, this->line_height_);

  bool build_start_face = false;
  if (!this->last_shift_)
  {
    this->setColor(color.r, color.g, color.b, color.a);
    this->last_shift_ = shift;
    build_start_face = true;
  }

  std::pair<Ogre::Vector3, Ogre::ColourValue> sp =
      std::make_pair(*this->last_point_ + *this->last_shift_, *this->last_color_);
  std::pair<Ogre::Vector3, Ogre::ColourValue> sm =
      std::make_pair(*this->last_point_ - *this->last_shift_, *this->last_color_);
  std::pair<Ogre::Vector3, Ogre::ColourValue> spz = std::make_pair(sp.first + z_shift, *this->last_color_);
  std::pair<Ogre::Vector3, Ogre::ColourValue> smz = std::make_pair(sm.first + z_shift, *this->last_color_);

  std::pair<Ogre::Vector3, Ogre::ColourValue> ep = std::make_pair(point + shift, color);
  std::pair<Ogre::Vector3, Ogre::ColourValue> em = std::make_pair(point - shift, color);
  std::pair<Ogre::Vector3, Ogre::ColourValue> epz = std::make_pair(ep.first + z_shift, color);
  std::pair<Ogre::Vector3, Ogre::ColourValue> emz = std::make_pair(em.first + z_shift, color);

  this->last_shift_ = shift;
  this->last_point_ = point;
  this->last_color_ = color;

  if (build_start_face)
  {
    // left
    auto start = { sp, sm, smz, spz };
    Ogre::Vector3 normal = (spz.first - sm.first).crossProduct(sm.first - sp.first).normalisedCopy();
    manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP,
                          "rviz_rendering");
    manual_object_->normal(normal);

    for (auto element : start)
    {
      manual_object_->position(element.first);
      manual_object_->colour(element.second);
    }
    manual_object_->end();
  }

  // top
  auto up = { epz, spz, emz, smz };
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP, "rviz_rendering");
  manual_object_->normal(Ogre::Vector3(0, 0, 1));

  for (auto element : up)
  {
    manual_object_->position(element.first);
    manual_object_->colour(element.second);
  }
  manual_object_->end();

  auto down = { sp, ep, sm, em };
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP, "rviz_rendering");
  manual_object_->normal(Ogre::Vector3(0, 0, -1));

  for (auto element : down)
  {
    manual_object_->position(element.first);
    manual_object_->colour(element.second);
  }
  manual_object_->end();

  // left
  auto right = { ep, sp, epz, spz };
  Ogre::Vector3 right_normal = (spz.first - sp.first).crossProduct(sp.first - ep.first).normalisedCopy();
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP, "rviz_rendering");
  manual_object_->normal(-right_normal);

  for (auto element : right)
  {
    manual_object_->position(element.first);
    manual_object_->colour(element.second);
  }
  manual_object_->end();

  // right
  auto left = { sm, em, smz, emz };
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP, "rviz_rendering");
  manual_object_->normal(right_normal);

  for (auto element : left)
  {
    manual_object_->position(element.first);
    manual_object_->colour(element.second);
  }
  manual_object_->end();
}

void LineObject::setVisible(bool visible)
{
  scene_node_->setVisible(visible, true);
}

void LineObject::setPosition(const Ogre::Vector3& position)
{
  scene_node_->setPosition(position);
}

void LineObject::setOrientation(const Ogre::Quaternion& orientation)
{
  scene_node_->setOrientation(orientation);
}

void LineObject::setScale(const Ogre::Vector3& scale)
{
  scene_node_->setScale(scale);
}

void LineObject::setLineWidth(const float line_width)
{
  line_width_ = line_width;
}

void LineObject::setLineHeight(const float line_height)
{
  this->line_height_ = line_height;
}

const Ogre::Vector3& LineObject::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& LineObject::getOrientation()
{
  return scene_node_->getOrientation();
}

const float& LineObject::getLineHeight()
{
  return this->line_height_;
}

const float& LineObject::getLineWidth()
{
  return this->line_width_;
}

void LineObject::setUserData(const Ogre::Any& data)
{
  manual_object_->getUserObjectBindings().setUserAny(data);
}

void LineObject::setColor(float r, float g, float b, float a)
{
  Ogre::ColourValue c(r, g, b, a);
  this->manual_object_material_->getTechnique(0)->setAmbient(c * 0.5);
  this->manual_object_material_->getTechnique(0)->setDiffuse(c);

  rviz_rendering::MaterialManager::enableAlphaBlending(this->manual_object_material_, c.a);
}

}  // namespace ssl_vehicle_tracking_visualization