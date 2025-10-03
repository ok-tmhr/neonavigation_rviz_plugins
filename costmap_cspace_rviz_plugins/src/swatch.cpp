/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include "costmap_cspace_rviz_plugins/swatch.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderable.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_rendering/custom_parameter_indices.hpp"

namespace costmap_cspace_rviz_plugins
{
namespace displays
{

// Helper class to set alpha parameter on all renderables.
class AlphaSetter : public Ogre::Renderable::Visitor
{
public:
  explicit AlphaSetter(float alpha)
  : alpha_vec_(alpha, alpha, alpha, alpha)
  {}

  void visit(Ogre::Renderable* rend, ushort /*lodIndex*/, bool /*isDebug*/, Ogre::Any* /*pAny*/ = nullptr) override
  {
    rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha_vec_);
  }

private:
  Ogre::Vector4 alpha_vec_;
};

size_t Swatch::material_count_ = 0;
size_t Swatch::map_count_ = 0;
size_t Swatch::node_count_ = 0;
size_t Swatch::texture_count_ = 0;

Swatch::Swatch(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node,
  size_t x, size_t y, size_t width, size_t height,
  float resolution, bool draw_under)
: scene_manager_(scene_manager),
  parent_scene_node_(parent_scene_node),
  manual_object_(nullptr),
  x_(x), y_(y), width_(width), height_(height)
{
  setupMaterial();
  setupSceneNodeWithManualObject();

  scene_node_->setPosition(x * resolution, y * resolution, 0);
  scene_node_->setScale(width * resolution, height * resolution, 1.0);

  if (draw_under) {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible(false);
}

Swatch::~Swatch()
{
  scene_manager_->destroyManualObject(manual_object_);
}

void Swatch::updateAlpha(
  const Ogre::SceneBlendType & sceneBlending, bool depth_write, float alpha)
{
  material_->setSceneBlending(sceneBlending);
  material_->setDepthWriteEnabled(depth_write);
  if (manual_object_) {
    AlphaSetter alpha_setter(alpha);
    manual_object_->visitRenderables(&alpha_setter);
  }
}

void Swatch::updateData(const costmap_cspace_msgs::msg::CSpace3D & map, const costmap_cspace_msgs::msg::CSpace3DUpdate& map_update, const int yaw)
{
  size_t pixels_size = width_ * height_;
  size_t map_size = map.data.size();
  size_t map_width = map.info.width;
  size_t map_height = map.info.height;

  auto pixels = std::vector<unsigned char>(pixels_size, 255);

  unsigned char* ptr = pixels.data();
  const int shift_map = yaw * map_width * map_height;
  unsigned int fw = map_width;
  for (unsigned int yy = y_; yy < y_ + height_; yy++)
  {
    int index = yy * fw + x_;
    int pixels_to_copy = std::min(width_, map_size - index);
    memcpy(ptr, &map.data[shift_map + index], pixels_to_copy);
    ptr += pixels_to_copy;
    if (index + pixels_to_copy >= pixels_size)
      break;
  }

  const size_t update_y_min = map_update.y;
  const size_t update_y_max = map_update.y + map_update.height;
  const size_t update_x_min = map_update.x;
  const size_t update_x_max = map_update.x + map_update.width;
  const size_t shift_update = map_update.width * map_update.height * yaw;
  const int8_t* const update_buf = map_update.data.data() + shift_update;
  if ((update_x_min < x_ + width_) && (x_ < update_x_max) && (update_y_min < y_ + height_) && (y_ < update_y_max))
  {
    for (unsigned int yy = std::max(y_, update_y_min); yy < std::min(y_ + height_, update_y_max); ++yy)
    {
      unsigned int xx = std::max(x_, update_x_min);
      unsigned char* to = pixels.data() + (yy - y_) * width_ + xx - x_;
      const unsigned char* from = reinterpret_cast<const unsigned char*>(update_buf + (yy - update_y_min) * map_update.width + (xx - update_x_min));
      for (; xx < std::min(x_ + width_, update_x_max); ++xx, ++to, ++from)
      {
        *to = std::max(*to, *from);
      }
    }
  }

  Ogre::DataStreamPtr pixel_stream(new Ogre::MemoryDataStream(pixels.data(), pixels_size));

  resetTexture(pixel_stream);
  resetOldTexture();
}

void Swatch::setVisible(bool visible)
{
  if (manual_object_) {
    manual_object_->setVisible(visible);
  }
}

void Swatch::resetOldTexture()
{
  if (old_texture_) {
    Ogre::TextureManager::getSingleton().remove(old_texture_);
    old_texture_.reset();
  }
}

void Swatch::setRenderQueueGroup(uint8_t group)
{
  if (manual_object_) {
    manual_object_->setRenderQueueGroup(group);
  }
}

void Swatch::setDepthWriteEnabled(bool depth_write_enabled)
{
  if (material_) {
    material_->setDepthWriteEnabled(depth_write_enabled);
  }
}

Ogre::Pass * Swatch::getTechniquePass()
{
  if (material_) {
    return material_->getTechnique(0)->getPass(0);
  }
  return nullptr;
}

std::string Swatch::getTextureName()
{
  if (texture_) {
    return texture_->getName();
  }
  return "";
}

void Swatch::resetTexture(Ogre::DataStreamPtr & pixel_stream)
{
  old_texture_ = texture_;

  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
    "CSpace3DMapTexture" + std::to_string(texture_count_++),
    "rviz_rendering",
    pixel_stream,
    static_cast<uint16_t>(width_), static_cast<uint16_t>(height_),
    Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);
}

void Swatch::setupMaterial()
{
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone("CSpaceMapMaterial" + std::to_string(material_count_++));

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);
}

void Swatch::setupSceneNodeWithManualObject()
{
  manual_object_ = scene_manager_->createManualObject("CSpaceMapObject" + std::to_string(map_count_++));

  scene_node_ = parent_scene_node_->createChildSceneNode(
    "CSpaceNodeObject" + std::to_string(node_count_++));
  scene_node_->attachObject(manual_object_);

  setupSquareManualObject();
}

void Swatch::setupSquareManualObject()
{
  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");

  // first triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);
  addPointWithPlaneCoordinates(0.0f, 1.0f);

  // second triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);

  manual_object_->end();
}

void Swatch::addPointWithPlaneCoordinates(float x, float y)
{
  manual_object_->position(x, y, 0.0f);
  manual_object_->textureCoord(x, y);
  manual_object_->normal(0.0f, 0.0f, 1.0f);
}

}  // namespace displays
}  // namespace costmap_cspace_rviz_plugins
