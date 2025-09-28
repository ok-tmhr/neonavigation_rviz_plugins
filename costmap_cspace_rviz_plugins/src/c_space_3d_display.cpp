/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

// This file is based on https://bit.ly/3RNiYJt

#include <boost/bind.hpp>
#include <algorithm>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreSharedPtr.h>

#include <rclcpp/rclcpp.hpp>

#include <costmap_cspace_rviz_plugins/c_space_3d_display.h>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/custom_parameter_indices.hpp>
#include <rviz_rendering/objects/grid.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/validate_floats.hpp>
// #include <rviz_common/validate_quaternions.hpp>
#include <rviz_common/display_context.hpp>

namespace costmap_cspace_rviz_plugins
{
// helper class to set alpha parameter on all renderables.
class AlphaSetter : public Ogre::Renderable::Visitor
{
public:
  explicit AlphaSetter(float alpha)
    : alpha_vec_(alpha, alpha, alpha, alpha)
  {
  }

  void visit(Ogre::Renderable* rend, ushort /*lodIndex*/, bool /*isDebug*/, Ogre::Any* /*pAny*/ = nullptr) override
  {
    rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha_vec_);
  }

private:
  Ogre::Vector4 alpha_vec_;
};

Swatch::Swatch(CSpace3DDisplay* parent, unsigned int x, unsigned int y, unsigned int width, unsigned int height,
               float resolution)
  : parent_(parent)
  , manual_object_(nullptr)
  , x_(x)
  , y_(y)
  , width_(width)
  , height_(height)
{
  // Set up map material
  static int material_count = 0;
  std::stringstream ss;
  ss << "CSpaceMapMaterial" << material_count++;
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone(ss.str());

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "CSpaceMapObject" << map_count++;
  manual_object_ = parent_->scene_manager_->createManualObject(ss2.str());

  static int node_count = 0;
  std::stringstream ss3;
  ss3 << "CSpaceNodeObject" << node_count++;

  scene_node_ = parent_->scene_node_->createChildSceneNode(ss3.str());
  scene_node_->attachObject(manual_object_);

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(1.0f, 1.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, 1.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(1.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(1.0f, 1.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }
  }
  manual_object_->end();

  scene_node_->setPosition(x * resolution, y * resolution, 0);
  scene_node_->setScale(width * resolution, height * resolution, 1.0);

  if (parent_->draw_under_property_->getValue().toBool())
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible(false);
}

Swatch::~Swatch()
{
  parent_->scene_manager_->destroyManualObject(manual_object_);
}

void Swatch::updateAlpha(const Ogre::SceneBlendType sceneBlending, bool depthWrite,
                         costmap_cspace_rviz_plugins::AlphaSetter* alpha_setter)
{
  material_->setSceneBlending(sceneBlending);
  material_->setDepthWriteEnabled(depthWrite);
  if (manual_object_)
  {
    manual_object_->visitRenderables(alpha_setter);
  }
}

void Swatch::updateData(const int yaw)
{
  unsigned int pixels_size = width_ * height_;
  int8_t* pixels = new int8_t[pixels_size];
  memset(pixels, 255, pixels_size);
  int8_t* ptr = pixels;
  const int N = parent_->current_map_.data.size();
  const int shift_map = yaw * parent_->current_map_.info.width * parent_->current_map_.info.height;
  unsigned int fw = parent_->current_map_.info.width;
  for (unsigned int yy = y_; yy < y_ + height_; yy++)
  {
    int index = yy * fw + x_;
    int pixels_to_copy = std::min(static_cast<int>(width_), N - index);
    memcpy(ptr, &parent_->current_map_.data[shift_map + index], pixels_to_copy);
    ptr += pixels_to_copy;
    if (index + pixels_to_copy >= N)
      break;
  }

  const unsigned int update_y_min = parent_->current_update_.y;
  const unsigned int update_y_max = parent_->current_update_.y + parent_->current_update_.height;
  const unsigned int update_x_min = parent_->current_update_.x;
  const unsigned int update_x_max = parent_->current_update_.x + parent_->current_update_.width;
  const size_t shift_update = parent_->current_update_.width * parent_->current_update_.height * yaw;
  const int8_t* const update_buf = parent_->current_update_.data.data() + shift_update;
  if ((update_x_min < x_ + width_) && (x_ < update_x_max) && (update_y_min < y_ + height_) && (y_ < update_y_max))
  {
    for (unsigned int yy = std::max(y_, update_y_min); yy < std::min(y_ + height_, update_y_max); ++yy)
    {
      unsigned int xx = std::max(x_, update_x_min);
      int8_t* to = pixels + (yy - y_) * width_ + xx - x_;
      const int8_t* from = update_buf + (yy - update_y_min) * parent_->current_update_.width + (xx - update_x_min);
      for (; xx < std::min(x_ + width_, update_x_max); ++xx, ++to, ++from)
      {
        *to = std::max(*to, *from);
      }
    }
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));

  if (!texture_.isNull())
  {
    Ogre::TextureManager::getSingleton().remove(texture_->getName());
    texture_.setNull();
  }

  static int tex_count = 0;
  std::stringstream ss;
  ss << "CSpace3DMapTexture" << tex_count++;
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, width_, height_, Ogre::PF_L8,
      Ogre::TEX_TYPE_2D, 0);

  delete[] pixels;
}

CSpace3DDisplay::CSpace3DDisplay()
  : rviz_common::Display()
  , loaded_(false)
  , resolution_(0.0f)
  , width_(0)
  , height_(0)
{
  connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));
  topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Topic", "", QString::fromStdString(rosidl_generator_traits::data_type<costmap_cspace_msgs::msg::CSpace3D>()),
      "costmap_cspace_msgs::msg::CSpace3D topic to subscribe to.", this, SLOT(updateTopic()));

  topic_update_property_ = new rviz_common::properties::RosTopicProperty(
      "Update Topic", "", QString::fromStdString(rosidl_generator_traits::data_type<costmap_cspace_msgs::msg::CSpace3DUpdate>()),
      "costmap_cspace_msgs::msg::CSpace3D topic to subscribe to.", this, SLOT(updateTopic()));

  alpha_property_ =
      new rviz_common::properties::FloatProperty("Alpha", 0.7, "Amount of transparency to apply to the map.", this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  color_scheme_property_ = new rviz_common::properties::EnumProperty("Color Scheme", "costmap", "How to color the occupancy values.", this,
                                                  SLOT(updatePalette()));
  // Option values here must correspond to indices in palette_textures_ array in onInitialize() below.
  color_scheme_property_->addOption("costmap", 0);
  color_scheme_property_->addOption("raw", 1);

  yaw_property_ = new rviz_common::properties::IntProperty("Yaw", 0, "Yaw number of drawn map.", this, SLOT(updateYaw()));
  yaw_property_->setMin(0);

  draw_under_property_ = new Property("Draw Behind", false,
                                      "Rendering option, controls whether or not the map is always drawn behind "
                                      "everything else.",
                                      this, SLOT(updateDrawUnder()));

  resolution_property_ = new rviz_common::properties::FloatProperty("Resolution", 0, "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  angular_resolution_property_ =
      new rviz_common::properties::FloatProperty("Angle Resolution", 0, "Angle resolution of the map. (not editable)", this);
  angular_resolution_property_->setReadOnly(true);

  width_property_ = new rviz_common::properties::IntProperty("Width", 0, "Width of the map, in meters. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new rviz_common::properties::IntProperty("Height", 0, "Height of the map, in meters. (not editable)", this);
  height_property_->setReadOnly(true);

  position_property_ =
      new rviz_common::properties::VectorProperty("Position", Ogre::Vector3::ZERO,
                               "Position of the bottom left corner of the map, in meters. (not editable)", this);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                                                       "Orientation of the map. (not editable)", this);
  orientation_property_->setReadOnly(true);

  unreliable_property_ = new BoolProperty("Unreliable", false, "Prefer UDP topic transport", this, SLOT(updateTopic()));

  transform_timestamp_property_ = new BoolProperty("Use Timestamp", false, "Use map header timestamp when transforming",
                                                   this, SLOT(transformMap()));
}

CSpace3DDisplay::~CSpace3DDisplay()
{
  unsubscribe();
  clear();
  for (unsigned i = 0; i < swatches_.size(); i++)
  {
    delete swatches_[i];
  }
  swatches_.clear();
}

unsigned char* makeCostmapPalette()
{
  unsigned char* palette = OGRE_ALLOC_T(unsigned char, 256 * 4, Ogre::MEMCATEGORY_GENERAL);
  unsigned char* palette_ptr = palette;

  // zero values have alpha=0
  *palette_ptr++ = 0;  // red
  *palette_ptr++ = 0;  // green
  *palette_ptr++ = 0;  // blue
  *palette_ptr++ = 0;  // alpha

  // Blue to red spectrum for most normal cost values
  for (int i = 1; i <= 99; i++)
  {
    unsigned char v = (255 * i) / 100;
    *palette_ptr++ = v;        // redcre
    *palette_ptr++ = 0;        // green
    *palette_ptr++ = 255 - v;  // blue
    *palette_ptr++ = 255;      // alpha
  }
  // lethal obstacle values (100) in purple
  *palette_ptr++ = 255;  // red
  *palette_ptr++ = 0;    // green
  *palette_ptr++ = 255;  // blue
  *palette_ptr++ = 255;  // alpha
  // illegal positive values in green
  for (int i = 101; i <= 127; i++)
  {
    *palette_ptr++ = 0;    // red
    *palette_ptr++ = 255;  // green
    *palette_ptr++ = 0;    // blue
    *palette_ptr++ = 255;  // alpha
  }
  // illegal negative (char) values in shades of red/yellow
  for (int i = 128; i <= 254; i++)
  {
    *palette_ptr++ = 255;                              // red
    *palette_ptr++ = (255 * (i - 128)) / (254 - 128);  // green
    *palette_ptr++ = 0;                                // blue
    *palette_ptr++ = 255;                              // alpha
  }
  // legal -1 value is tasteful blueish greenish grayish color
  *palette_ptr++ = 0x70;  // red
  *palette_ptr++ = 0x89;  // green
  *palette_ptr++ = 0x86;  // blue
  *palette_ptr++ = 255;   // alpha

  return palette;
}

unsigned char* makeRawPalette()
{
  unsigned char* palette = OGRE_ALLOC_T(unsigned char, 256 * 4, Ogre::MEMCATEGORY_GENERAL);
  unsigned char* palette_ptr = palette;
  // Standard gray map palette values
  for (int i = 0; i < 256; i++)
  {
    *palette_ptr++ = i;    // red
    *palette_ptr++ = i;    // green
    *palette_ptr++ = i;    // blue
    *palette_ptr++ = 255;  // alpha
  }

  return palette;
}

Ogre::TexturePtr makePaletteTexture(unsigned char* palette_bytes)
{
  Ogre::DataStreamPtr palette_stream;
  palette_stream.bind(new Ogre::MemoryDataStream(palette_bytes, 256 * 4, true));

  static int palette_tex_count = 0;
  std::stringstream ss;
  ss << "CSpace3DMapPaletteTexture" << palette_tex_count++;
  return Ogre::TextureManager::getSingleton().loadRawData(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, palette_stream, 256, 1, Ogre::PF_BYTE_RGBA,
      Ogre::TEX_TYPE_1D, 0);
}

void CSpace3DDisplay::onInitialize()
{
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
  color_scheme_transparency_.push_back(true);
  palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
  color_scheme_transparency_.push_back(true);
}

void CSpace3DDisplay::onEnable()
{
  subscribe();
}

void CSpace3DDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void CSpace3DDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  current_map_ = costmap_cspace_msgs::msg::CSpace3D();
  current_update_ = costmap_cspace_msgs::msg::CSpace3DUpdate();

  if (!topic_property_->getTopic().isEmpty())
  {
    try
    {
      auto update_nh_ = rviz_ros_node_.lock()->get_raw_node();
      using std::placeholders::_1;
      if (unreliable_property_->getBool())
      {
        map_sub_ = update_nh_->create_subscription<costmap_cspace_msgs::msg::CSpace3D>(topic_property_->getTopicStd(), rclcpp::QoS(1).best_effort().transient_local(), std::bind(&CSpace3DDisplay::incomingMap, this, _1));
      }
      else
      {
        map_sub_ = update_nh_->create_subscription<costmap_cspace_msgs::msg::CSpace3D>(topic_property_->getTopicStd(), rclcpp::QoS(1).reliable().transient_local(), std::bind(&CSpace3DDisplay::incomingMap, this, _1));
      }
      setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    }
    catch (rclcpp::exceptions::InvalidTopicNameError& e)
    {
      setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }

    if (!topic_update_property_->getTopic().isEmpty())
    {
      try
      {
        auto update_nh_ = rviz_ros_node_.lock()->get_raw_node();
        using std::placeholders::_1;
        update_sub_ =
            update_nh_->create_subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>(topic_update_property_->getTopicStd(), 1, std::bind(&CSpace3DDisplay::incomingUpdate, this, _1));
        setStatus(rviz_common::properties::StatusProperty::Ok, "Update Topic", "OK");
      }
      catch (rclcpp::exceptions::InvalidTopicNameError& e)
      {
        setStatus(rviz_common::properties::StatusProperty::Error, "Update Topic", QString("Error subscribing: ") + e.what());
      }
    }
    else
    {
      setStatus(rviz_common::properties::StatusProperty::Ok, "Update Topic", QString("Not specified"));
    }
  }
}

void CSpace3DDisplay::unsubscribe()
{
  map_sub_.reset();
  update_sub_.reset();
}

void CSpace3DDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::SceneBlendType sceneBlending;
  bool depthWrite;

  if (alpha < 0.9998 || color_scheme_transparency_[color_scheme_property_->getOptionInt()])
  {
    sceneBlending = Ogre::SBT_TRANSPARENT_ALPHA;
    depthWrite = false;
  }
  else
  {
    sceneBlending = Ogre::SBT_REPLACE;
    depthWrite = !draw_under_property_->getValue().toBool();
  }

  AlphaSetter alpha_setter(alpha);

  for (unsigned i = 0; i < swatches_.size(); i++)
  {
    swatches_[i]->updateAlpha(sceneBlending, depthWrite, &alpha_setter);
  }
}

void CSpace3DDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= 0.9998)
  {
    for (unsigned i = 0; i < swatches_.size(); i++)
      swatches_[i]->material_->setDepthWriteEnabled(!draw_under);
  }

  int group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
  for (unsigned i = 0; i < swatches_.size(); i++)
  {
    if (swatches_[i]->manual_object_)
      swatches_[i]->manual_object_->setRenderQueueGroup(group);
  }
}

void CSpace3DDisplay::updateYaw()
{
  const int yaw = yaw_property_->getInt();
  for (unsigned i = 0; i < swatches_.size(); i++)
  {
    swatches_[i]->updateData(yaw);
  }
  Q_EMIT mapUpdated();
}

void CSpace3DDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  clear();
}

void CSpace3DDisplay::clear()
{
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");

  if (!loaded_)
  {
    return;
  }

  for (unsigned i = 0; i < swatches_.size(); i++)
  {
    if (swatches_[i]->manual_object_)
      swatches_[i]->manual_object_->setVisible(false);

    if (!swatches_[i]->texture_.isNull())
    {
      Ogre::TextureManager::getSingleton().remove(swatches_[i]->texture_->getName());
      swatches_[i]->texture_.setNull();
    }
  }

  loaded_ = false;
}

bool validateFloats(const costmap_cspace_msgs::msg::CSpace3D& msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.info.linear_resolution);
  valid = valid && rviz_common::validateFloats(msg.info.origin);
  return valid;
}

void CSpace3DDisplay::incomingMap(const costmap_cspace_msgs::msg::CSpace3D::ConstPtr& msg)
{
  current_map_ = *msg;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
  loaded_ = true;
}

void CSpace3DDisplay::incomingUpdate(const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstPtr& update)
{
  // Only update the map if we have gotten a full one first.
  if (!loaded_)
  {
    return;
  }

  // Reject updates which have any out-of-bounds data.
  if (update->x < 0 || update->y < 0 || current_map_.info.width < update->x + update->width ||
      current_map_.info.height < update->y + update->height)
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Update", "Update area outside of original map area.");
    return;
  }

  current_update_ = *update;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

void CSpace3DDisplay::createSwatches()
{
  int width = current_map_.info.width;
  int height = current_map_.info.height;
  float resolution = current_map_.info.linear_resolution;

  int sw = width;
  int sh = height;
  int n_swatches = 1;

  for (int i = 0; i < 4; i++)
  {
    RVIZ_COMMON_LOG_INFO_STREAM("Creating" << n_swatches << "swatches");
    for (unsigned i = 0; i < swatches_.size(); i++)
    {
      delete swatches_[i];
    }
    swatches_.clear();
    try
    {
      int x = 0;
      int y = 0;
      for (int i = 0; i < n_swatches; i++)
      {
        int tw, th;
        if (width - x - sw >= sw)
          tw = sw;
        else
          tw = width - x;

        if (height - y - sh >= sh)
          th = sh;
        else
          th = height - y;

        swatches_.push_back(new Swatch(this, x, y, tw, th, resolution));
        swatches_[i]->updateData(0);

        x += tw;
        if (x >= width)
        {
          x = 0;
          y += sh;
        }
      }
      updateAlpha();
      return;
    }
    catch (Ogre::RenderingAPIException&)
    {
      RVIZ_COMMON_LOG_WARNING_STREAM("Failed to create " << n_swatches << "swatches");
      if (sw > sh)
        sw /= 2;
      else
        sh /= 2;
      n_swatches *= 2;
    }
  }
}

void CSpace3DDisplay::showMap()
{
  if (current_map_.data.empty())
  {
    return;
  }
  yaw_property_->setMax(current_map_.info.angle - 1);

  if (!validateFloats(current_map_))
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Map", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // if (!rviz_common::properties::validateQuaternions(current_map_.info.origin))
  // {
  //   RVIZ_COMMON_LOG_WARNING_ONCE_NAMED("quaternions",
  //                       "Map received on topic '%s' contains unnormalized quaternions. "
  //                       "This warning will only be output once but may be true for others; "
  //                       "enable DEBUG messages for ros.rviz.quaternions to see more details.",
  //                       topic_property_->getTopicStd().c_str());
  //   RVIZ_COMMON_LOG_DEBUG_NAMED("quaternions", "Map received on topic '%s' contains unnormalized quaternions.",
  //                   topic_property_->getTopicStd().c_str());
  // }

  if (current_map_.info.width * current_map_.info.height == 0)
  {
    std::stringstream ss;
    ss << "Map is zero-sized (" << current_map_.info.width << "x" << current_map_.info.height << ")";
    setStatus(rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
    return;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");

  RVIZ_COMMON_LOG_DEBUG_STREAM("Received a " << current_map_.info.width << " X " << current_map_.info.height << " map @ " << current_map_.info.linear_resolution << " m/pix\n");

  const float resolution = current_map_.info.linear_resolution;
  const float anglular_resolution = current_map_.info.angular_resolution;

  const int width = current_map_.info.width;
  const int height = current_map_.info.height;
  const int angle = current_map_.info.angle;

  if (width != width_ || height != height_ || resolution_ != resolution)
  {
    createSwatches();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
  }

  Ogre::Vector3 position(current_map_.info.origin.position.x, current_map_.info.origin.position.y,
                         current_map_.info.origin.position.z);
  Ogre::Quaternion orientation;
  // rviz_common::properties::normalizeQuaternion(current_map_.info.origin.orientation, orientation);

  frame_ = current_map_.header.frame_id;
  if (frame_.empty())
  {
    frame_ = "map";
  }

  bool map_status_set = false;
  if (width * height * angle != static_cast<int>(current_map_.data.size()))
  {
    std::stringstream ss;
    ss << "Data size doesn't match width*height: width = " << width << ", height = " << height
       << ", data size = " << current_map_.data.size();
    setStatus(rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
    map_status_set = true;
  }

  const int yaw = yaw_property_->getInt();
  for (size_t i = 0; i < swatches_.size(); i++)
  {
    swatches_[i]->updateData(yaw);

    Ogre::Pass* pass = swatches_[i]->material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 0)
    {
      tex_unit = pass->getTextureUnitState(0);
    }
    else
    {
      tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(swatches_[i]->texture_->getName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);
    swatches_[i]->manual_object_->setVisible(true);
  }

  if (!map_status_set)
  {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Map", "Map OK");
  }
  updatePalette();

  resolution_property_->setValue(resolution);
  width_property_->setValue(width);
  height_property_->setValue(height);
  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);
  angular_resolution_property_->setValue(anglular_resolution);

  transformMap();

  context_->queueRender();
}

void CSpace3DDisplay::updatePalette()
{
  int palette_index = color_scheme_property_->getOptionInt();

  for (unsigned i = 0; i < swatches_.size(); i++)
  {
    Ogre::Pass* pass = swatches_[i]->material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* palette_tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 1)
    {
      palette_tex_unit = pass->getTextureUnitState(1);
    }
    else
    {
      palette_tex_unit = pass->createTextureUnitState();
    }
    palette_tex_unit->setTextureName(palette_textures_[palette_index]->getName());
    palette_tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  }

  updateAlpha();
}

void CSpace3DDisplay::transformMap()
{
  if (!loaded_)
  {
    return;
  }

  rclcpp::Time transform_time;

  if (transform_timestamp_property_->getBool())
  {
    transform_time = current_map_.header.stamp;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(frame_, transform_time, current_map_.info.origin, position,
                                              orientation) &&
      !context_->getFrameManager()->transform(frame_, rclcpp::Time(0), current_map_.info.origin, position, orientation))
  {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming map '" << qPrintable(getName()) << "' from frame '" + frame_ + "' to frame '" + qPrintable(fixed_frame_) + "'");

    setStatus(rviz_common::properties::StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame_) + "] to [" + fixed_frame_ + "]");
  }
  else
  {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void CSpace3DDisplay::fixedFrameChanged()
{
  transformMap();
}

void CSpace3DDisplay::reset()
{
  Display::reset();

  clear();
  // Force resubscription so that the map will be re-sent
  updateTopic();
}

void CSpace3DDisplay::setTopic(const QString& topic, const QString& /*datatype*/)
{
  topic_property_->setString(topic);
}

void CSpace3DDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  transformMap();
}

}  // namespace costmap_cspace_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(costmap_cspace_rviz_plugins::CSpace3DDisplay, rviz_common::Display)
