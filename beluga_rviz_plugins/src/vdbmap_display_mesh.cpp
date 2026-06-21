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

  // Convert volume to mesh
  std::vector<openvdb::Vec3s> points;
  std::vector<openvdb::Vec3I> triangles;
  std::vector<openvdb::Vec4I> quads;
  openvdb::tools::volumeToMesh(*grid, points, triangles, quads);

  // Convert to Ogre mesh
  Ogre::MeshPtr mesh = convertOpenVDBToOgreMesh("VDBMesh", points, triangles, quads);

  // Create entity
  Ogre::Entity* entity = context_->getSceneManager()->createEntity("VDBEntity", "VDBMesh");

  // Create scene node and attach entity
  Ogre::SceneNode* node = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  node->attachObject(entity);
  entity->setMaterialName("VoxelColor");
}

Ogre::MeshPtr VdbMapDisplay::convertOpenVDBToOgreMesh(
    const std::string& meshName,
    const std::vector<openvdb::Vec3s>& points,
    const std::vector<openvdb::Vec3I>& triangles,
    const std::vector<openvdb::Vec4I>& quads) {
  // Create the mesh
  Ogre::MeshPtr mesh =
      Ogre::MeshManager::getSingleton().createManual(meshName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

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
      decl->getVertexSize(0), points.size(), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  // Fill vertex buffer
  float* pVertex = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  for (const auto& point : points) {
    *pVertex++ = point.x();
    *pVertex++ = point.y();
    *pVertex++ = point.z();
  }

  vbuf->unlock();

  // Set vertex buffer
  mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vbuf);

  // Calculate index count (3 indices per triangle, 6 per quad)
  size_t indexCount = triangles.size() * 3 + quads.size() * 6;
  Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_32BIT, indexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  // Fill index buffer
  Ogre::uint32* pIndices = static_cast<Ogre::uint32*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  size_t idx = 0;

  // Add triangles
  for (const auto& tri : triangles) {
    pIndices[idx++] = tri[0];
    pIndices[idx++] = tri[1];
    pIndices[idx++] = tri[2];
  }

  // Add quads (as two triangles)
  for (const auto& quad : quads) {
    // First triangle
    pIndices[idx++] = quad[0];
    pIndices[idx++] = quad[1];
    pIndices[idx++] = quad[2];

    // Second triangle
    pIndices[idx++] = quad[0];
    pIndices[idx++] = quad[2];
    pIndices[idx++] = quad[3];
  }

  ibuf->unlock();

  // Set index buffer
  subMesh->indexData->indexBuffer = ibuf;
  subMesh->indexData->indexCount = indexCount;
  subMesh->indexData->indexStart = 0;

  // Set mesh bounds
  Ogre::AxisAlignedBox aabb;
  for (const auto& point : points) {
    aabb.merge(Ogre::Vector3(point.x(), point.y(), point.z()));
  }
  mesh->_setBounds(aabb);
  mesh->_setBoundingSphereRadius(aabb.getHalfSize().length());

  // Load mesh
  mesh->load();

  return mesh;
}

}  // namespace beluga_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(beluga_rviz_plugins::VdbMapDisplay, rviz_common::Display)
