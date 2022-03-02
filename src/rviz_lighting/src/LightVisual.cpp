/** @file LightVisual.cpp
 *
 *  This software was created by Terry Welsh for the Intelligent Robotics Group
 *  at NASA Ames Research Center. Copies and derivatives of this file must
 *  retain this paragraph. There are no other restrictions on the use of this
 *  software.
 *
 *  @author Terry Welsh (terence.m.welsh@nasa.gov)
 */

#include <OGRE/OgreSceneManager.h>
#include "LightVisual.h"


namespace rviz_lighting
{


LightVisual::LightVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  light_ = scene_manager_->createLight();
  light_->setType( Ogre::Light::LT_DIRECTIONAL );
  // Initial values for color, direction, position, and other things are set when their respective slots get called
}

LightVisual::~LightVisual()
{
  scene_manager_->destroyLight(light_);
}

void LightVisual::setEnabled(bool enabled)
{
  light_->setVisible(enabled);
}

void LightVisual::setLightType(int type)
{
  light_->setType(Ogre::Light::LightTypes(type));
}

void LightVisual::setDiffuseColor(float r, float g, float b)
{
  light_->setDiffuseColour(r, g, b);
}

void LightVisual::setSpecularColor(float r, float g, float b)
{
  light_->setSpecularColour(r, g, b);
}

void LightVisual::setDirection(const Ogre::Vector3& dir)
{
  light_->setDirection(dir);
}

void LightVisual::setPosition(const Ogre::Vector3& pos)
{
  light_->setPosition(pos);
}

void LightVisual::setOuterAngle(float angle)
{
  light_->setSpotlightOuterAngle(Ogre::Radian(angle));
}

void LightVisual::setFalloff(float falloff)
{
  light_->setSpotlightFalloff(falloff);
}


} // end namespace rviz_lighting
