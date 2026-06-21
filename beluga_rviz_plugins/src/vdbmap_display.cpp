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
#include <OgreInstanceManager.h>  // For InstanceManager class
#include <OgreInstancedEntity.h>  // For InstancedEntity class
#include <OgreManualObject.h>
#include <OgreMeshManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreStaticGeometry.h>
#include <OgreSubMesh.h>

#include <openvdb/Grid.h>
#include <openvdb/io/File.h>
#include <openvdb/tools/VolumeToMesh.h>

#include <rviz_common/logging.hpp>

#include "beluga_rviz_plugins/vdbmap_display.hpp"

namespace beluga_rviz_plugins {

VdbMapDisplay::VdbMapDisplay() {
  openvdb::initialize();

  update_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Update Topic", "", "",
      "Topic where updates to this map display are received. "
      "This topic is automatically determined by the map topic. "
      "If the map is received on 'map_topic', the display assumes updates are received on "
      "'map_topic_updates'."
      "This can be overridden in the UI by clicking on the topic and setting the desired topic.",
      this, SLOT(updateMapUpdateTopic()));

  // Set properties
  color_property_ =
      new rviz_common::properties::ColorProperty("Color", QColor(255, 0, 0), "Color to draw the voxels", this);

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.7f, "Transparency of the voxels", this);
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  map_path_property_ = new rviz_common::properties::StringProperty(
      "Path", "/ws/src/beluga/beluga_rviz_plugins/test/bunny.vdb", "Path of the VDB file", this);
}

VdbMapDisplay::~VdbMapDisplay() {}

void VdbMapDisplay::onInitialize() {
  // Create voxel material color
  const std::string material_name = "VoxelColor";
  voxel_color_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
  // Set the material to be red
  voxel_color_->setDiffuse(1.0f, 0.0f, 0.0f, 1.0f);
  voxel_color_->setAmbient(1.0f, 0.0f, 0.0f);

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
  // const openvdb::Vec3d voxel_size = grid->transform().voxelSize();
  float voxel_size = 1;
  Ogre::Vector3 position(0.5f, 0.5f, 0.5f);
  Ogre::Vector3 position2(-0.5f, -0.5f, -0.5f);
  std::vector<Ogre::Vector3> points;
  points.push_back(position);
  points.push_back(position2);

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
      "MyInstancedMaterial",  // Your material name
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setDiffuse(1.0, 0.0, 0.0, 1.0);  // Example: Red color

  static int objCounter = 0;
  std::string manualObjName = "VoxelManualObject_" + std::to_string(objCounter++);
  Ogre::ManualObject* man = context_->getSceneManager()->createManualObject(manualObjName);
  man->begin("MyInstancedMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  const float half_size = voxel_size / 2.0f;
  // Front face vertices (z = -half_size)
  man->position(-half_size, -half_size, -half_size);  // 0
  man->position(half_size, -half_size, -half_size);   // 1
  man->position(half_size, half_size, -half_size);    // 2
  man->position(-half_size, half_size, -half_size);   // 3

  // Back face vertices (z = half_size)
  man->position(-half_size, -half_size, half_size);  // 4
  man->position(half_size, -half_size, half_size);   // 5
  man->position(half_size, half_size, half_size);    // 6
  man->position(-half_size, half_size, half_size);   // 7

  // Define the 12 triangles (6 faces × 2 triangles each)
  // Front face
  man->triangle(0, 1, 2);
  man->triangle(0, 2, 3);

  // Back face
  man->triangle(5, 4, 7);
  man->triangle(5, 7, 6);

  // Left face
  man->triangle(4, 0, 3);
  man->triangle(4, 3, 7);

  // Right face
  man->triangle(1, 5, 6);
  man->triangle(1, 6, 2);

  // Bottom face
  man->triangle(4, 5, 1);
  man->triangle(4, 1, 0);

  // Top face
  man->triangle(3, 2, 6);
  man->triangle(3, 6, 7);
  man->end();

  std::string meshName = "VoxelMesh_" + std::to_string(objCounter++);
  Ogre::MeshPtr mesh = man->convertToMesh(meshName);
  // Set mesh bounds (CRITICAL)
  Ogre::AxisAlignedBox aabb(-half_size, -half_size, -half_size, half_size, half_size, half_size);
  mesh->_setBounds(aabb);
  mesh->_setBoundingSphereRadius(half_size * 1.732f);  // sqrt(3) for diagonal

  Ogre::InstanceManager* instanceManager = context_->getSceneManager()->createInstanceManager(
      "InstanceMgrName",  // Unique name
      meshName,           // Mesh to instance
      Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME,
      Ogre::InstanceManager::HWInstancingBasic,  // Technique
      points.size(),                             // Number of instances (can grow later)
      Ogre::IM_USE16BIT                          // Use 16-bit indices if possible
  );

  // Set position/rotation/scale for this instance
  for (const auto& point : points) {
    Ogre::InstancedEntity* instancedEntity = instanceManager->createInstancedEntity("MyInstancedMaterial");
    context_->getSceneManager()->getRootSceneNode()->createChildSceneNode()->attachObject(instancedEntity);
    instancedEntity->setPosition(point);
  }
  instanceManager->defragmentBatches(true);

  // Attach to scene node
  // instanceManager->setBatchesAsStaticAndUpdate(true);
}

}  // namespace beluga_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(beluga_rviz_plugins::VdbMapDisplay, rviz_common::Display)
