/** @file AmbientLightDisplay.h
 *
 *  This software was created by Terry Welsh for the Intelligent Robotics Group
 *  at NASA Ames Research Center. Copies and derivatives of this file must
 *  retain this paragraph. There are no other restrictions on the use of this
 *  software.
 *
 *  @author Terry Welsh (terence.m.welsh@nasa.gov)
 */

#ifndef AMBIENT_LIGHT_DISPLAY_H
#define AMBIENT_LIGHT_DISPLAY_H

#include <rviz/display.h>


namespace rviz
{
class ColorProperty;
}

namespace rviz_lighting
{


/**
 * \class AmbientLightDisplay
 * \brief This class wraps OGRE::SceneManager::setAmbientLight()
 *
 * \todo: make it so you can add only one AmbientLight to an rviz config
 */
class AmbientLightDisplay : public rviz::Display
{
Q_OBJECT
public:
  // RViz requires plugins to use default constructor with no arguments
  AmbientLightDisplay();
  virtual ~AmbientLightDisplay();

protected:
  virtual void onInitialize() override;

  virtual void onEnable() override;

  virtual void onDisable() override;

  virtual void reset() override;

private Q_SLOTS:
  void updateColor();

private:
  // User-editable property variables
  rviz::ColorProperty* color_property_;
};


} // end namespace rviz_lighting

#endif
