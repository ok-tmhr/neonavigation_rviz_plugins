/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2025, Tomohiro Oku
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

#include "costmap_cspace_rviz_plugins/c_space_3d_display.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include "rclcpp/time.hpp"

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "costmap_cspace_rviz_plugins/palette_builder.hpp"


namespace costmap_cspace_rviz_plugins
{
namespace displays
{

CSpace3DDisplay::CSpace3DDisplay()
: loaded_(false),
  resolution_(0.0f),
  width_(0),
  height_(0),
  update_profile_(rclcpp::QoS(5)),
  update_messages_received_(0)
{
  connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));

  update_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Update Topic", "",
    "", "Topic where updates to this map display are received. "
    "This topic is automatically determined by the map topic. "
    "If the map is received on 'map_topic', the display assumes updates are received on "
    "'map_topic_updates'."
    "This can be overridden in the UI by clicking on the topic and setting the desired topic.",
    this, SLOT(updateMapUpdateTopic()));

  update_profile_property_ = new rviz_common::properties::QosProfileProperty(
    update_topic_property_, update_profile_);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.7f,
    "Amount of transparency to apply to the map.",
    this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  color_scheme_property_ = new rviz_common::properties::EnumProperty(
    "Color Scheme", "map",
    "How to color the occupancy values.",
    this, SLOT(updatePalette()));
  // Option values here must correspond to indices in palette_textures_ array in onInitialize()
  // below.
  color_scheme_property_->addOption("map", 0);
  color_scheme_property_->addOption("costmap", 1);
  color_scheme_property_->addOption("raw", 2);

  yaw_property_ = new rviz_common::properties::IntProperty("Yaw", 0, "Yaw number of drawn map.", this, SLOT(updateYaw()));
  yaw_property_->setMin(0);

  draw_under_property_ = new rviz_common::properties::BoolProperty(
    "Draw Behind", false,
    "Rendering option, controls whether or not the map is always"
    " drawn behind everything else.",
    this, SLOT(updateDrawUnder()));

  resolution_property_ = new rviz_common::properties::FloatProperty(
    "Resolution", 0,
    "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  angular_resolution_property_ =
      new rviz_common::properties::FloatProperty("Angle Resolution", 0, "Angle resolution of the map. (not editable)", this);
  angular_resolution_property_->setReadOnly(true);

  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 0,
    "Width of the map, in meters. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 0,
    "Height of the map, in meters. (not editable)", this);
  height_property_->setReadOnly(true);

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Position of the bottom left corner of the map, in meters. (not editable)",
    this);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, "Orientation of the map. (not editable)", this);
  orientation_property_->setReadOnly(true);

  transform_timestamp_property_ = new rviz_common::properties::BoolProperty(
    "Use Timestamp", false,
    "Use map header timestamp when transforming", this, SLOT(transformMap()));
}

CSpace3DDisplay::~CSpace3DDisplay()
{
  unsubscribe();
  clear();
}

Ogre::TexturePtr makePaletteTexture(std::vector<unsigned char> palette_bytes)
{
  Ogre::DataStreamPtr palette_stream;
  palette_stream.reset(new Ogre::MemoryDataStream(palette_bytes.data(), 256 * 4));

  static int palette_tex_count = 0;
  std::string tex_name = "CSpace3DMapPaletteTexture" + std::to_string(palette_tex_count++);
  return Ogre::TextureManager::getSingleton().loadRawData(
    tex_name, "rviz_rendering", palette_stream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);
}

CSpace3DDisplay::CSpace3DDisplay(rviz_common::DisplayContext * context)
: CSpace3DDisplay()
{
  context_ = context;
  scene_manager_ = context->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  palette_textures_.push_back(makePaletteTexture(makeMapPalette()));
  color_scheme_transparency_.push_back(false);
  palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
  color_scheme_transparency_.push_back(true);
  palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
  color_scheme_transparency_.push_back(true);
}

void CSpace3DDisplay::onInitialize()
{
  MFDClass::onInitialize();
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  update_topic_property_->initialize(rviz_ros_node_);

  update_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->update_profile_ = profile;
      updateMapUpdateTopic();
    });
  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back(makePaletteTexture(makeMapPalette()));
  color_scheme_transparency_.push_back(false);
  palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
  color_scheme_transparency_.push_back(true);
  palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
  color_scheme_transparency_.push_back(true);
}

void CSpace3DDisplay::updateTopic()
{
  update_topic_property_->setValue(topic_property_->getTopic() + "_updates");
  MFDClass::updateTopic();
}

void CSpace3DDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  if (topic_property_->isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic",
      QString("Error subscribing: Empty topic name"));
    return;
  }

  MFDClass::subscribe();

  subscribeToUpdateTopic();
}

void CSpace3DDisplay::subscribeToUpdateTopic()
{
  try {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.event_callbacks.message_lost_callback =
      [&](rclcpp::QOSMessageLostInfo & info)
      {
        std::ostringstream sstm;
        sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
          info.total_count_change << " \n>\tTotal number of messages lost: " <<
          info.total_count;
        setStatus(
          rviz_common::properties::StatusProperty::Warn, "Update Topic",
          QString(sstm.str().c_str()));
      };

    rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
    update_subscription_ =
      node->
      template create_subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>(
      update_topic_property_->getTopicStd(), update_profile_,
      [this](const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr message) {
        incomingUpdate(message);
      },
      sub_opts);
    subscription_start_time_ = node->now();
    setStatus(rviz_common::properties::StatusProperty::Ok, "Update Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Update Topic",
      QString("Error subscribing: ") + e.what());
  }
}

void CSpace3DDisplay::unsubscribe()
{
  MFDClass::unsubscribe();
  unsubscribeToUpdateTopic();
}

void CSpace3DDisplay::unsubscribeToUpdateTopic()
{
  update_subscription_.reset();
}

void CSpace3DDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::SceneBlendType scene_blending;
  bool depth_write;

  rviz_rendering::MaterialManager::enableAlphaBlending(scene_blending, depth_write, alpha);

  for (const auto & swatch : swatches_) {
    swatch->updateAlpha(scene_blending, depth_write, alpha);
  }
}

void CSpace3DDisplay::updateDrawUnder() const
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= rviz_rendering::unit_alpha_threshold) {
    for (const auto & swatch : swatches_) {
      swatch->setDepthWriteEnabled(!draw_under);
    }
  }

  uint8_t group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
  for (const auto & swatch : swatches_) {
    swatch->setRenderQueueGroup(group);
  }
}

void CSpace3DDisplay::updateYaw()
{
  const int yaw = yaw_property_->getInt();
  for (unsigned i = 0; i < swatches_.size(); i++)
  {
    swatches_[i]->updateData(current_map_, current_update_, yaw);
  }
  Q_EMIT mapUpdated();
}

void CSpace3DDisplay::clear()
{
  if (isEnabled()) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");
  }

  if (!loaded_) {
    return;
  }

  swatches_.clear();
  height_ = 0;
  width_ = 0;
  resolution_ = 0.0f;

  loaded_ = false;
}

bool validateFloats(const costmap_cspace_msgs::msg::CSpace3D & msg)
{
  return rviz_common::validateFloats(msg.info.linear_resolution) &&
         rviz_common::validateFloats(msg.info.origin);
}

void CSpace3DDisplay::processMessage(costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr msg)
{
  current_map_ = *msg;
  loaded_ = true;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

void CSpace3DDisplay::incomingUpdate(const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr update)
{
  // Only update the map if we have gotten a full one first.
  if (!loaded_) {
    return;
  }

  ++update_messages_received_;
  QString topic_str = QString::number(messages_received_) + " update messages received";
  // Append topic subscription frequency if we can lock rviz_ros_node_.
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_interface =
    rviz_ros_node_.lock();
  if (node_interface != nullptr) {
    const double duration =
      (node_interface->get_raw_node()->now() - subscription_start_time_).seconds();
    const double subscription_frequency =
      static_cast<double>(messages_received_) / duration;
    topic_str += " at " + QString::number(subscription_frequency, 'f', 1) + " hz.";
  }
  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Topic",
    topic_str);

  if (updateDataOutOfBounds(update)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Update", "Update area outside of original map area.");
    return;
  }

  current_update_ = *update;
  updateMapDataInMemory(update);
  setStatus(rviz_common::properties::StatusProperty::Ok, "Update", "Update OK");

  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

bool CSpace3DDisplay::updateDataOutOfBounds(
  const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr update) const
{
  return current_map_.info.width < update->x + update->width ||
         current_map_.info.height < update->y + update->height;
}

void CSpace3DDisplay::updateMapDataInMemory(
  const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr update)
{
  for (size_t y = 0; y < update->height; y++) {
    auto offset = update->data.begin() + y * update->width;
    std::copy(
      offset,
      offset + update->width,
      current_map_.data.begin() + (update->y + y) * current_map_.info.width + update->x);
  }
}

void CSpace3DDisplay::createSwatches()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;
  float resolution = current_map_.info.linear_resolution;

  size_t swatch_width = width;
  size_t swatch_height = height;
  int number_swatches = 1;
  // One swatch can have up to 2^16 * 2^16 pixel (8 bit texture, i.e. 4GB of data)
  // Since the width and height are separately limited by 2^16 it might be necessary to have several
  // pieces, however more than 8 swatches is probably unnecessary due to memory limitations
  const size_t maximum_number_swatch_splittings = 4;

  for (size_t i = 0; i < maximum_number_swatch_splittings; ++i) {
    RVIZ_COMMON_LOG_INFO_STREAM(
      "Trying to create a map of size " <<
        width << " x " << height << " using " << number_swatches << " swatches");
    swatches_.clear();
    try {
      tryCreateSwatches(width, height, resolution, swatch_width, swatch_height, number_swatches);
      updateDrawUnder();
      return;
    } catch (Ogre::InvalidParametersException &) {
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    } catch (Ogre::RenderingAPIException &) {
      // This exception seems no longer thrown on some systems. May still be relevant for others.
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    }
  }
  RVIZ_COMMON_LOG_ERROR_STREAM(
    "Creating " << number_swatches << "failed. "
      "This map is too large to be displayed by RViz.");
  swatches_.clear();
}

void CSpace3DDisplay::doubleSwatchNumber(
  size_t & swatch_width, size_t & swatch_height, int & number_swatches) const
{
  RVIZ_COMMON_LOG_ERROR_STREAM(
    "Failed to create map using " << number_swatches << " swatches. "
      "At least one swatch seems to need too much memory");
  if (swatch_width > swatch_height) {
    swatch_width /= 2;
  } else {
    swatch_height /= 2;
  }
  number_swatches *= 2;
}

void CSpace3DDisplay::tryCreateSwatches(
  size_t width,
  size_t height,
  float resolution,
  size_t swatch_width,
  size_t swatch_height,
  int number_swatches)
{
  size_t x = 0;
  size_t y = 0;
  for (int i = 0; i < number_swatches; i++) {
    size_t effective_width = getEffectiveDimension(width, swatch_width, x);
    size_t effective_height = getEffectiveDimension(height, swatch_height, y);

    swatches_.push_back(
      std::make_shared<Swatch>(
        scene_manager_,
        scene_node_,
        x, y,
        effective_width,
        effective_height,
        resolution,
        draw_under_property_->getValue().toBool()));

    auto yaw = yaw_property_->getInt();
    swatches_[i]->updateData(current_map_, current_update_, yaw);

    x += effective_width;
    if (x >= width) {
      x = 0;
      y += effective_height;
    }
  }
  updateAlpha();
}

size_t CSpace3DDisplay::getEffectiveDimension(
  size_t map_dimension, size_t swatch_dimension, size_t position)
{
  // Last swatch is bigger than swatch_dimension for odd numbers.
  // subtracting the swatch_dimension in the LHS handles this case.
  return map_dimension - position - swatch_dimension >= swatch_dimension ?
         swatch_dimension :
         map_dimension - position;
}

void CSpace3DDisplay::showMap()
{
  if (current_map_.data.empty()) {
    return;
  }
  yaw_property_->setMax(current_map_.info.angle - 1);

  if (!validateFloats(current_map_)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Map",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;
  size_t angle = current_map_.info.angle;

  if (width * height == 0) {
    std::string message =
      "Map is zero-sized (" + std::to_string(width) + "x" + std::to_string(height) + ")";
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(message));
    return;
  }

  if (width * height * angle != current_map_.data.size()) {
    std::string message =
      "Data size doesn't match width*height: width = " + std::to_string(width) + ", height = " +
      std::to_string(height) + ", data size = " + std::to_string(current_map_.data.size());
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(message));
    return;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "Received a " << current_map_.info.width << " X " <<
      current_map_.info.height << " map @ " << current_map_.info.linear_resolution << "m/pix\n");

  showValidMap();
}

void CSpace3DDisplay::showValidMap()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;

  float resolution = current_map_.info.linear_resolution;
  const float anglular_resolution = current_map_.info.angular_resolution;

  resetSwatchesIfNecessary(width, height, resolution);

  frame_ = current_map_.header.frame_id;
  if (frame_.empty()) {
    frame_ = "map";
  }

  updateSwatches();

  setStatus(rviz_common::properties::StatusProperty::Ok, "Map", "Map OK");
  updatePalette();

  resolution_property_->setValue(resolution);
  width_property_->setValue(static_cast<unsigned int>(width));
  height_property_->setValue(static_cast<unsigned int>(height));

  position_property_->setVector(rviz_common::pointMsgToOgre(current_map_.info.origin.position));
  orientation_property_->setQuaternion(
    rviz_common::quaternionMsgToOgre(current_map_.info.origin.orientation));
  angular_resolution_property_->setValue(anglular_resolution);

  transformMap();

  updateDrawUnder();

  context_->queueRender();
}

void CSpace3DDisplay::resetSwatchesIfNecessary(size_t width, size_t height, float resolution)
{
  if (width != width_ || height != height_ || resolution_ != resolution) {
    createSwatches();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
  }
}

void CSpace3DDisplay::updateSwatches() const
{
  const int yaw = yaw_property_->getInt();
  for (const auto & swatch : swatches_) {
    swatch->updateData(current_map_, current_update_, yaw);

    Ogre::Pass * pass = swatch->getTechniquePass();
    Ogre::TextureUnitState * tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 0) {
      tex_unit = pass->getTextureUnitState(0);
    } else {
      tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(swatch->getTextureName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);
    swatch->setVisible(true);
    swatch->resetOldTexture();
  }
}

void CSpace3DDisplay::updatePalette()
{
  int palette_index = color_scheme_property_->getOptionInt();

  for (const auto & swatch : swatches_) {
    Ogre::Pass * pass = swatch->getTechniquePass();
    Ogre::TextureUnitState * palette_tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 1) {
      palette_tex_unit = pass->getTextureUnitState(1);
    } else {
      palette_tex_unit = pass->createTextureUnitState();
    }
    palette_tex_unit->setTexture(palette_textures_[palette_index]);
    palette_tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  }

  updateAlpha();
  updateDrawUnder();
}

void CSpace3DDisplay::transformMap()
{
  if (!loaded_) {
    return;
  }

  rclcpp::Time transform_time = context_->getClock()->now();

  if (transform_timestamp_property_->getBool()) {
    transform_time = current_map_.header.stamp;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(
      frame_, transform_time, current_map_.info.origin, position, orientation) &&
    !context_->getFrameManager()->transform(
      frame_, rclcpp::Time(0, 0, context_->getClock()->get_clock_type()),
      current_map_.info.origin, position, orientation))
  {
    setMissingTransformToFixedFrame(frame_);
    scene_node_->setVisible(false);
  } else {
    setTransformOk();

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
  }
}

void CSpace3DDisplay::fixedFrameChanged()
{
  transformMap();
}

void CSpace3DDisplay::reset()
{
  MFDClass::reset();
  update_messages_received_ = 0;
  clear();
}

void CSpace3DDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  transformMap();
}

void CSpace3DDisplay::onEnable()
{
  MFDClass::onEnable();
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");
}

void CSpace3DDisplay::updateMapUpdateTopic()
{
  unsubscribeToUpdateTopic();
  reset();
  subscribeToUpdateTopic();
  context_->queueRender();
}

}  // namespace displays
}  // namespace costmap_cspace_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(costmap_cspace_rviz_plugins::displays::CSpace3DDisplay, rviz_common::Display)
