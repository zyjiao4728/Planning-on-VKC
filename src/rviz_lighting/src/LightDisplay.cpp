/** @file LightDisplay.cpp
 *
 *  This software was created by Terry Welsh for the Intelligent Robotics Group
 *  at NASA Ames Research Center. Copies and derivatives of this file must
 *  retain this paragraph. There are no other restrictions on the use of this
 *  software.
 *
 *  @author Terry Welsh (terence.m.welsh@nasa.gov)
 */

#include "LightDisplay.h"
#include "LightVisual.h"

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <OgreLight.h>


namespace rviz_lighting
{


using namespace rviz;
using namespace std;


LightDisplay::LightDisplay()
  : Display()
  , frame_position_(0, 0, 0)
{
  frame_property_ = new TfFrameProperty("Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
                                        "The TF frame to which this light is attached. The frame position will have no effect on Directional lights. The frame orientation will have no effect on Point lights.",
                                        this, NULL, true);

  light_type_property_ = new EnumProperty("Light Type", "Directional",
                                          "Which type of light",
                                          this, SLOT(updateLightType()));
  light_type_property_->addOption("Directional", Ogre::Light::LT_DIRECTIONAL);
  light_type_property_->addOption("Point", Ogre::Light::LT_POINT);
  light_type_property_->addOption("Spotlight", Ogre::Light::LT_SPOTLIGHT);

  diffuse_property_ = new ColorProperty("Diffuse Color", QColor(255, 255, 255),
                                        "Diffuse color of the light.",
                                        this, SLOT(updateDiffuseColor()));

  specular_property_ = new ColorProperty("Specular Color", QColor(255, 255, 255),
                                         "Specular color of the light. RViz materials do not use specular, so this will have no effect except maybe when loading a mesh with materials with specular.",
                                         this, SLOT(updateSpecularColor()));

  direction_property_ = new VectorProperty("Direction", Ogre::Vector3(-1, -1, -1),
                                           "Direction of the light.",
                                           this, SLOT(updateDirection()));

  position_property_ = new VectorProperty("Position", Ogre::Vector3(0, 0, 0),
                                          "Position of the light.",
                                          this, SLOT(updatePosition()));

  outer_angle_property_ = new FloatProperty("Outer Angle", 0.5f,
                                            "Angle covered by the spotlight's outer cone (radians).",
                                            this, SLOT(updateOuterAngle()));

  falloff_property_ = new FloatProperty("Falloff", 1.0f,
                                        "The rate of falloff between the inner and outer cones. 1.0 means a linear falloff, less means slower falloff, higher means faster falloff.",
                                        this, SLOT(updateFalloff()));
}

LightDisplay::~LightDisplay()
{
}

void LightDisplay::update(float wall_dt, float ros_dt)
{
  QString qframe = frame_property_->getFrame();
  std::string frame = qframe.toStdString();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if(context_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation))
  {
    frame_position_ = position;
    frame_orientation_ = orientation;
    updatePosition();
    updateDirection();
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }
  else
  {
    std::string error;
    if(context_->getFrameManager()->transformHasProblems(frame, ros::Time(), error))
    {
      setStatus(StatusProperty::Error, "Transform", QString::fromStdString(error));
    }
    else
    {
      setStatus(StatusProperty::Error,
                "Transform",
                "Could not transform from [" + qframe + "] to Fixed Frame [" + fixed_frame_ + "] for an unknown reason" );
    }
  }
}

void LightDisplay::onInitialize()
{
  Display::onInitialize();

  frame_property_->setFrameManager(context_->getFrameManager());

  visual_.reset(new LightVisual(context_->getSceneManager(), scene_node_));

  // Show and hide the appropriate properties for the current type of light
  updateLightType();
}

void LightDisplay::onEnable()
{
  visual_->setEnabled(true);
}

void LightDisplay::onDisable()
{
  visual_->setEnabled(false);
}

void LightDisplay::reset()
{
  Display::reset();
}

void LightDisplay::updateLightType()
{
  switch(light_type_property_->getOptionInt())
  {
  case Ogre::Light::LT_DIRECTIONAL:
    direction_property_->setHidden(false);
    position_property_->setHidden(true);
    outer_angle_property_->setHidden(true);
    falloff_property_->setHidden(true);
    break;
  case Ogre::Light::LT_POINT:
    direction_property_->setHidden(true);
    position_property_->setHidden(false);
    outer_angle_property_->setHidden(true);
    falloff_property_->setHidden(true);
    break;
  case Ogre::Light::LT_SPOTLIGHT:
    direction_property_->setHidden(false);
    position_property_->setHidden(false);
    outer_angle_property_->setHidden(false);
    falloff_property_->setHidden(false);
    break;
  }

  visual_->setLightType(light_type_property_->getOptionInt());
}

void LightDisplay::updateDiffuseColor()
{
  Ogre::ColourValue color = diffuse_property_->getOgreColor();
  visual_->setDiffuseColor(color.r, color.g, color.b);
}

void LightDisplay::updateSpecularColor()
{
  Ogre::ColourValue color = specular_property_->getOgreColor();
  visual_->setSpecularColor(color.r, color.g, color.b);
}

void LightDisplay::updateDirection()
{
  Ogre::Vector3 direction = direction_property_->getVector();
  visual_->setDirection(frame_orientation_ * direction);
}

void LightDisplay::updatePosition()
{
  Ogre::Vector3 position = position_property_->getVector();
  visual_->setPosition(position + frame_position_);
}

void LightDisplay::updateOuterAngle()
{
  visual_->setOuterAngle(outer_angle_property_->getFloat());
}

void LightDisplay::updateFalloff()
{
  visual_->setFalloff(falloff_property_->getFloat());
}


} // end namespace rviz_lighting


// This must be in the global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_lighting::LightDisplay, rviz::Display)
