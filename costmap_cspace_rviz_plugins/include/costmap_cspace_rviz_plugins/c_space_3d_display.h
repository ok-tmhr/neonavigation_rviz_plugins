/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2022, the neonavigation authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This file is based on https://bit.ly/3yV4zC2

#ifndef COSTMAP_CSPACE_RVIZ_PLUGINS_C_SPACE_3D_DISPLAY_H
#define COSTMAP_CSPACE_RVIZ_PLUGINS_C_SPACE_3D_DISPLAY_H

#include <vector>
#include <string>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreSharedPtr.h>
#endif

#include <costmap_cspace_msgs/msg/c_space3_d.hpp>
#include <costmap_cspace_msgs/msg/c_space3_d_update.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <rclcpp/time.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace Ogre
{
class ManualObject;
}  // namespace Ogre

namespace rviz_common::properties
{
class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class RosTopicProperty;
class VectorProperty;
class BoolProperty;
}  // namespace rviz

namespace costmap_cspace_rviz_plugins
{
class CSpace3DDisplay;
class AlphaSetter;

class Swatch
{
  friend class CSpace3DDisplay;

public:
  Swatch(CSpace3DDisplay* parent, unsigned int x, unsigned int y, unsigned int width, unsigned int height,
         float resolution);
  ~Swatch();
  void updateAlpha(const Ogre::SceneBlendType sceneBlending, bool depthWrite,
                   costmap_cspace_rviz_plugins::AlphaSetter* alpha_setter);
  void updateData(const int yaw);

protected:
  CSpace3DDisplay* parent_;
  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  Ogre::SceneNode* scene_node_;
  unsigned int x_, y_, width_, height_;
};

/**
 * \class MapDisplay
 * \brief Displays a map along the XY plane.
 */
class CSpace3DDisplay : public rviz_common::Display
{
  friend class Swatch;
  Q_OBJECT
public:
  CSpace3DDisplay();
  ~CSpace3DDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;

  float getResolution()
  {
    return resolution_;
  }
  int getWidth()
  {
    return width_;
  }
  int getHeight()
  {
    return height_;
  }

  void setTopic(const QString& topic, const QString& datatype) override;

Q_SIGNALS:
  /** @brief Emitted when a new map is received*/
  void mapUpdated();

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();
  void updatePalette();
  void updateYaw();
  /** @brief Show current_map_ in the scene. */
  void showMap();
  void transformMap();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();
  void update(float wall_dt, float ros_dt) override;

  /** @brief Copy msg into current_map_ and call showMap(). */
  void incomingMap(const costmap_cspace_msgs::msg::CSpace3D::ConstPtr& msg);

  /** @brief Copy update's data into current_map_ and call showMap(). */
  void incomingUpdate(const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstPtr& update);
  void clear();
  void createSwatches();

  std::vector<Swatch*> swatches_;
  std::vector<Ogre::TexturePtr> palette_textures_;
  std::vector<bool> color_scheme_transparency_;
  bool loaded_;

  std::string topic_;
  float resolution_;
  int width_;
  int height_;
  int angle_;
  std::string frame_;
  costmap_cspace_msgs::msg::CSpace3D current_map_;
  costmap_cspace_msgs::msg::CSpace3DUpdate current_update_;

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr map_sub_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr update_sub_;

  rviz_common::properties::RosTopicProperty* topic_property_;
  rviz_common::properties::RosTopicProperty* topic_update_property_;
  rviz_common::properties::FloatProperty* resolution_property_;
  rviz_common::properties::FloatProperty* angular_resolution_property_;
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::VectorProperty* position_property_;
  rviz_common::properties::QuaternionProperty* orientation_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::Property* draw_under_property_;
  rviz_common::properties::EnumProperty* color_scheme_property_;
  rviz_common::properties::IntProperty* yaw_property_;

  rviz_common::properties::BoolProperty* unreliable_property_;
  rviz_common::properties::BoolProperty* transform_timestamp_property_;
};

}  // namespace costmap_cspace_rviz_plugins

#endif  // COSTMAP_CSPACE_RVIZ_PLUGINS_C_SPACE_3D_DISPLAY_H
