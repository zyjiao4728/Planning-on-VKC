/** @file LightVisual.h
 *
 *  This software was created by Terry Welsh for the Intelligent Robotics Group
 *  at NASA Ames Research Center. Copies and derivatives of this file must
 *  retain this paragraph. There are no other restrictions on the use of this
 *  software.
 *
 *  @author Terry Welsh (terence.m.welsh@nasa.gov)
 */

#ifndef LIGHT_VISUAL_H
#define LIGHT_VISUAL_H


namespace Ogre
{
class Light;
class Quaternion;
class Vector3;
}

namespace rviz_lighting
{


// This class represents a light in the scene
/**
 * \class LightVisual
 * \brief This class wraps Ogre::Light
 *
 * \todo Enable shadows. This probably requires mods to rviz because shadows
 * need to be enabled when SceneManager is initialized. So far texture shadows
 * cause RViz to crash, so it would take some work. Stencil shadows work better,
 * but they aren't the best option for most scenes.
 */
class LightVisual
{
public:
  LightVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  virtual ~LightVisual();

  void setEnabled(bool enabled);
  void setLightType(int type);
  void setDiffuseColor(float r, float g, float b);
  void setSpecularColor(float r, float g, float b);
  void setDirection(const Ogre::Vector3& dir);
  void setPosition(const Ogre::Vector3& pos);
  void setOuterAngle(float angle);
  void setFalloff(float falloff);

private:
  // The SceneManager, saved here so it can destroy the light_ in the destructor
  Ogre::SceneManager* scene_manager_;

  Ogre::Light* light_;
};


} // end namespace rviz_lighting

#endif
