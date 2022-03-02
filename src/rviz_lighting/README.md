# rviz_lighting
This is an RViz plugin that provides ambient, directional, point, and spot lights. These lights can be used to give an RViz scene a more interesting lighting environment. They can be attached to tf frames, mimicing lights attached to moving robot components.

### Making it work better with RViz
RViz is not compatible with extra lights for a few reasons. I use a custom RViz with the following changes to get the full effect of rviz_lighting:
* RViz has a default light attached to the camera which I have disabled.
* Made ambient component of material the same as diffuse instead of cutting it in half. This makes it so the full range of ambient light can be used and the developer will get the amount of ambient light they expect from their light settings.
* Added a specular component to materials, so specular light will have an effect.

Following are the exact changes I made to RViz. If anything like rviz_lighting were to be added to RViz's default plugins, changes like these should be made (or they should at least be made optional).
```sh
diff --git a/src/rviz/default_plugin/markers/mesh_resource_marker.cpp b/src/rviz/default_plugin/markers/mesh_resource_marker.cpp
index 9ea7470..7b5386b 100644
--- a/src/rviz/default_plugin/markers/mesh_resource_marker.cpp
+++ b/src/rviz/default_plugin/markers/mesh_resource_marker.cpp
@@ -229,8 +229,10 @@ void MeshResourceMarker::onNewMessage(const MarkerConstPtr& old_message, const M
     for (material_it = materials_.begin(); material_it != materials_.end(); material_it++)
     {
       Ogre::Technique* technique = (*material_it)->getTechnique(0);
-      technique->setAmbient( r*0.5, g*0.5, b*0.5 );
+      technique->setAmbient( r, g, b );
       technique->setDiffuse( r, g, b, a );
+      technique->setSpecular( 1, 1, 1, 1 );
+      technique->setShininess(20.0);
       technique->setSceneBlending( blending );
       technique->setDepthWriteEnabled( depth_write );
       technique->setLightingEnabled( true );
diff --git a/src/rviz/robot/robot_link.cpp b/src/rviz/robot/robot_link.cpp
index f4ff022..03fbb5b 100644
--- a/src/rviz/robot/robot_link.cpp
+++ b/src/rviz/robot/robot_link.cpp
@@ -387,6 +387,8 @@ void RobotLink::updateAlpha()
       Ogre::ColourValue color = material->getTechnique(0)->getPass(0)->getDiffuse();
       color.a = robot_alpha_ * material_alpha_ * link_alpha;
       material->setDiffuse( color );
+      material->setSpecular( 1, 1, 1, 1 );
+      material->setShininess(20.0);
 
       if ( color.a < 0.9998 )
       {
@@ -404,6 +406,8 @@ void RobotLink::updateAlpha()
   Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
   color.a = robot_alpha_ * link_alpha;
   color_material_->setDiffuse( color );
+  color_material_->setSpecular( 1, 1, 1, 1 );
+  color_material_->setShininess(20.0);
 
   if ( color.a < 0.9998 )
   {
@@ -471,8 +475,10 @@ Ogre::MaterialPtr RobotLink::getMaterialForLink( const urdf::LinkConstSharedPtr&
   if (visual->material->texture_filename.empty())
   {
     const urdf::Color& col = visual->material->color;
-    mat->getTechnique(0)->setAmbient(col.r * 0.5, col.g * 0.5, col.b * 0.5);
+    mat->getTechnique(0)->setAmbient(col.r, col.g, col.b);
     mat->getTechnique(0)->setDiffuse(col.r, col.g, col.b, col.a);
+    mat->getTechnique(0)->setSpecular(1, 1, 1, 1);
+    mat->getTechnique(0)->setShininess(20.0);
 
     material_alpha_ = col.a;
   }
@@ -926,8 +932,10 @@ void RobotLink::setColor( float red, float green, float blue )
   color.r = red;
   color.g = green;
   color.b = blue;
-  color_material_->getTechnique(0)->setAmbient( 0.5 * color );
+  color_material_->getTechnique(0)->setAmbient( color );
   color_material_->getTechnique(0)->setDiffuse( color );
+  color_material_->getTechnique(0)->setSpecular(1, 1, 1, 1);
+  color_material_->getTechnique(0)->setShininess(20.0);
 
   using_color_ = true;
   setToNormalMaterial();
diff --git a/src/rviz/visualization_manager.cpp b/src/rviz/visualization_manager.cpp
index 9367b08..53d870f 100644
--- a/src/rviz/visualization_manager.cpp
+++ b/src/rviz/visualization_manager.cpp
@@ -141,12 +141,12 @@ VisualizationManager::VisualizationManager( RenderPanel* render_panel, WindowMan
   scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );
 
   rviz::RenderSystem::RenderSystem::get()->prepareOverlays(scene_manager_);
-
+/*
   directional_light_ = scene_manager_->createLight( "MainDirectional" );
   directional_light_->setType( Ogre::Light::LT_DIRECTIONAL );
   directional_light_->setDirection( Ogre::Vector3( -1, 0, -1 ) );
   directional_light_->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );
-
+*/
   root_display_group_ = new DisplayGroup();
   root_display_group_->setName( "root" );
   display_property_tree_model_ = new PropertyTreeModel( root_display_group_ );
@@ -277,6 +277,8 @@ void createColorMaterial(const std::string& name, const Ogre::ColourValue& color
   Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create( name, ROS_PACKAGE_NAME );
   mat->setAmbient(color * 0.5f);
   mat->setDiffuse(color);
+  mat->setSpecular(1, 1, 1, 1);
+  mat->setShininess(20.0);
   if( use_self_illumination )
   {
     mat->setSelfIllumination(color);
@@ -355,7 +357,7 @@ void VisualizationManager::onUpdate()
         view_manager_->getCurrent() &&
         view_manager_->getCurrent()->getCamera() )
   {
-    directional_light_->setDirection(view_manager_->getCurrent()->getCamera()->getDerivedDirection());
+//    directional_light_->setDirection(view_manager_->getCurrent()->getCamera()->getDerivedDirection());
   }
 
   frame_count_++;
```

### To-do
* There are other Ogre::Light features that could be exposed.
* Shadows.
* Per-pixel lighting. This would require some fancy shader work and probably would not be part of this plugin.
