// Copyright 2025 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreStaticGeometry.h>

#include <openvdb/Grid.h>
#include <openvdb/io/File.h>
#include <openvdb/openvdb.h>

#include <rviz_common/logging.hpp>

#include "beluga_rviz_plugins/vdbmap_display.hpp"

namespace beluga_rviz_plugins {

VdbMapDisplay::VdbMapDisplay() {
  openvdb::initialize();

  // Set properties
  color_property_ =
      new rviz_common::properties::ColorProperty("Color", QColor(255, 0, 0), "Color to draw the voxels", this);

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.7f, "Transparency of the voxels", this);
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  map_path_property_ = new rviz_common::properties::StringProperty(
      "Path", "/ws/src/beluga/beluga_rviz_plugins/test/map_1005_07.vdb", "Path of the VDB file", this);
}

VdbMapDisplay::~VdbMapDisplay() {}

void VdbMapDisplay::onInitialize() {
  // Create voxel material color
  const std::string material_name = "VoxelColor";
  voxel_color_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
  loadMap();
}

void VdbMapDisplay::loadMap() {
  if (map_path_property_->getStdString().empty()) {
    return;
  }

  openvdb::io::File file(map_path_property_->getStdString());
  // Open the file.  This reads the file header, but not any grids.
  file.open();
  // Read the entire contents of the file and return a list of grid pointers.
  openvdb::GridPtrVecPtr grids = file.getGrids();
  // Close the file
  file.close();

  if (grids->empty()) {
    RVIZ_COMMON_LOG_WARNING("No grids found in VDB file");
    return;
  }

  // Cast the generic grid pointer to a FloatGrid pointer.
  openvdb::FloatGrid::Ptr grid;
  grid = openvdb::gridPtrCast<openvdb::FloatGrid>((*grids)[0]);
  const openvdb::Vec3d voxel_size = grid->transform().voxelSize();

  // Get display properties
  const QColor color = color_property_->getColor();
  const float alpha = alpha_property_->getFloat();
  voxel_color_->setDiffuse(color.redF(), color.greenF(), color.blueF(), alpha);
  voxel_color_->setAmbient(color.redF() * 0.5, color.greenF() * 0.5, color.blueF() * 0.5);

  // Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  // color.a = alpha_property_->getFloat();
  // rviz_rendering::MaterialManager::enableAlphaBlending(voxel_color_, color.a);

  Ogre::ManualObject* man = context_->getSceneManager()->createManualObject("test");
  man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Iterate through active voxels
  for (openvdb::FloatGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter) {
    const openvdb::Coord& coord = iter.getCoord();
    const openvdb::Vec3d world_pos = grid->indexToWorld(coord);
    cubeCreator(man, Ogre::Vector3(world_pos.x(), world_pos.y(), world_pos.z()), 0.07f);
  }

  man->end();
  context_->getSceneManager()->getRootSceneNode()->createChildSceneNode()->attachObject(man);
}

void VdbMapDisplay::cubeCreator(Ogre::ManualObject* man, const Ogre::Vector3& position, const float& resolution) {
  const float half_size = resolution / 2.0f;
  for (int i = 0; i < 8; ++i) {
    // Determine sign for x, y, z using bitwise operations
    float x = position.x + ((i & 1) ? half_size : -half_size);
    float y = position.y + ((i & 2) ? half_size : -half_size);
    float z = position.z + ((i & 4) ? half_size : -half_size);

    man->position(x, y, z);
    man->normal(0, 0, 1);
  }
}

}  // namespace beluga_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(beluga_rviz_plugins::VdbMapDisplay, rviz_common::Display)
