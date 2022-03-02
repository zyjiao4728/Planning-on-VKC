/** @file LightDisplay.h
 *
 *  This software was created by Terry Welsh for the Intelligent Robotics Group
 *  at NASA Ames Research Center. Copies and derivatives of this file must
 *  retain this paragraph. There are no other restrictions on the use of this
 *  software.
 *
 *  @author Terry Welsh (terence.m.welsh@nasa.gov)
 */

#ifndef LIGHT_DISPLAY_H
#define LIGHT_DISPLAY_H

#include <rviz/display.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>


namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class EnumProperty;
class FloatProperty;
class TfFrameProperty;
class VectorProperty;
}


namespace rviz_lighting
{


class LightVisual;

// This class represents a light in the scene
/**
 * \class LightDisplay
 * \brief This class represents a light in the scene (directional, point, or spot light)
 */
class LightDisplay : public rviz::Display
{
Q_OBJECT
public:
  // RViz requires plugins to use default constructor with no arguments
  LightDisplay();
  virtual ~LightDisplay();

  /** @brief Update position and direction of light (necessary if it is attached to a moving tf frame) */
  virtual void update(float wall_dt, float ros_dt);

protected:
  virtual void onInitialize() override;

  virtual void onEnable() override;

  virtual void onDisable() override;

  virtual void reset() override;

private Q_SLOTS:
  void updateLightType();
  void updateDiffuseColor();
  void updateSpecularColor();
  void updateDirection();
  void updatePosition();
  void updateOuterAngle();
  void updateFalloff();

private:
  // Wrapper around an Ogre::Light and anything else that affects the Ogre scene
  boost::shared_ptr<LightVisual> visual_;

  // User-editable property variables
  rviz::TfFrameProperty* frame_property_;
  rviz::EnumProperty* light_type_property_;
  rviz::ColorProperty* diffuse_property_;
  rviz::ColorProperty* specular_property_;
  rviz::VectorProperty* direction_property_;
  rviz::VectorProperty* position_property_;
  rviz::FloatProperty* outer_angle_property_;
  rviz::FloatProperty* falloff_property_;

  Ogre::Quaternion frame_orientation_;
  Ogre::Vector3 frame_position_;
};


} // end namespace rviz_lighting

#endif
