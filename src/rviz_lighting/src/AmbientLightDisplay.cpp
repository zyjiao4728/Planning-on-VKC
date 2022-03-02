/** @file AmbientLightDisplay.cpp
 *
 *  This software was created by Terry Welsh for the Intelligent Robotics Group
 *  at NASA Ames Research Center. Copies and derivatives of this file must
 *  retain this paragraph. There are no other restrictions on the use of this
 *  software.
 *
 *  @author Terry Welsh (terence.m.welsh@nasa.gov)
 */

#include "AmbientLightDisplay.h"
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <OGRE/OgreSceneManager.h>


namespace rviz_lighting
{

using namespace rviz;


AmbientLightDisplay::AmbientLightDisplay()
  : Display()
{
  color_property_ = new ColorProperty( "Color", QColor( 100, 100, 100 ),
                                             "Color of ambient light.",
                                             this, SLOT( updateColor() ));
}

AmbientLightDisplay::~AmbientLightDisplay()
{
}

void AmbientLightDisplay::onInitialize()
{
  Display::onInitialize();
}

void AmbientLightDisplay::onEnable()
{
  updateColor();
}

void AmbientLightDisplay::onDisable()
{
  updateColor();
}

void AmbientLightDisplay::reset()
{
  Display::reset();
}

void AmbientLightDisplay::updateColor()
{
  if (isEnabled())
  {
    Ogre::ColourValue color = color_property_->getOgreColor();
    context_->getSceneManager()->setAmbientLight(Ogre::ColourValue(color.r, color.g, color.b));
  }
  else
  {
    context_->getSceneManager()->setAmbientLight(Ogre::ColourValue(0.0f, 0.0f, 0.0f));
  }
}


} // end namespace rviz_lighting


// This must be in the global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_lighting::AmbientLightDisplay, rviz::Display )
