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

// Base indices for one cube (0-7)
const Ogre::uint32 cubeIndices[36] = {
    // Front face
    0, 1, 2, 1, 3, 2,
    // Back face
    5, 4, 7, 4, 6, 7,
    // Left face
    4, 0, 6, 0, 2, 6,
    // Right face
    1, 5, 3, 5, 7, 3,
    // Bottom face
    4, 5, 0, 5, 1, 0,
    // Top face
    2, 3, 6, 3, 7, 6};

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
  float voxel_size = 0.07;
  Ogre::Vector3 position(0.0f, 0.0f, 1.0f);
  Ogre::Vector3 position2(0.0f, 0.0f, 0.0f);
  std::vector<Ogre::Vector3> points;
  points.push_back(position);
  points.push_back(position2);

  // Create the mesh
  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
      "VDBMesh", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  // Create a submesh
  Ogre::SubMesh* subMesh = mesh->createSubMesh();
  subMesh->useSharedVertices = true;
  subMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;

  // Create vertex data structure
  mesh->sharedVertexData = new Ogre::VertexData();
  Ogre::VertexDeclaration* decl = mesh->sharedVertexData->vertexDeclaration;
  size_t offset = 0;

  // Position
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // Allocate vertex buffer
  Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      decl->getVertexSize(0), points.size() * 8, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  // Fill vertex buffer
  float* pVertex = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  const float half_size = voxel_size / 2.0f;
  for (const auto& point : points) {
    for (int i = 0; i < 8; ++i) {
      // Determine sign for x, y, z using bitwise operations
      *pVertex++ = point.x + ((i & 1) ? half_size : -half_size);
      *pVertex++ = point.y + ((i & 2) ? half_size : -half_size);
      *pVertex++ = point.z + ((i & 4) ? half_size : -half_size);
    }
  }

  vbuf->unlock();
  // Set vertex buffer
  mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vbuf);

  // Calculate index count
  size_t indexCount = points.size() * 36;
  Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_32BIT, indexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  // Fill index buffer
  Ogre::uint32* pIndices = static_cast<Ogre::uint32*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  size_t idx = 0;

  for (size_t cube = 0; cube < points.size(); ++cube) {
    const Ogre::uint32 vertexOffset = cube * 8;  // Each cube uses 8 vertices
    for (size_t i = 0; i < 36; ++i) {
      pIndices[cube * 36 + i] = cubeIndices[i] + vertexOffset;
    }
  }

  ibuf->unlock();

  // Set index buffer
  subMesh->indexData->indexBuffer = ibuf;
  subMesh->indexData->indexCount = indexCount;
  subMesh->indexData->indexStart = 0;

  // Set mesh bounds
  Ogre::AxisAlignedBox aabb;
  for (const auto& point : points) {
    aabb.merge(point);
  }
  mesh->_setBounds(aabb);
  mesh->_setBoundingSphereRadius(aabb.getHalfSize().length());

  // Load mesh
  mesh->load();

  // Create entity
  Ogre::Entity* entity = context_->getSceneManager()->createEntity("VDBEntity", "VDBMesh");
  entity->setMaterial(voxel_color_);

  // Create scene node and attach entity
  Ogre::SceneNode* node = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  node->attachObject(entity);
}

}  // namespace beluga_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(beluga_rviz_plugins::VdbMapDisplay, rviz_common::Display)
