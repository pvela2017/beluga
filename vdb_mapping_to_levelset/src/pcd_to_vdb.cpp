#include <openvdb/openvdb.h>
#include <openvdb/tree/Tree.h>
#include <openvdb/tools/TopologyToLevelSet.h>
#include <openvdb/math/Transform.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

int main()
{
    openvdb::initialize();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Open PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("scans.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded cloud succesfuly"<< std::endl;


    // Create VDB grid of resolution 0.5
    openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create();
    grid->setTransform(openvdb::math::Transform::createLinearTransform(0.5));
    grid->setGridClass(openvdb::GRID_LEVEL_SET);

    // Get an accessor for coordinate-based access to voxels.
    openvdb::FloatGrid::Accessor accessor = grid->getAccessor();

    // Fill the grid
    for (const auto& point: *cloud)
    {
        // World coordinates
        openvdb::Vec3f world_vect(point.x, point.y, point.z);

        // Transform to index world
        openvdb::math::Transform transform;
        openvdb::math::Coord ijk = transform.worldToIndexCellCentered(world_vect);

        // Set the voxel to 1
        accessor.setValue(ijk, 1.0);
        // Activate the voxel
        accessor.setActiveState(ijk);
    }

    // Transform to level set
    openvdb::FloatGrid::Ptr grid_levelset = nullptr;
    grid_levelset = openvdb::tools::topologyToLevelSet(*grid, 3, 1, 0, 0);

    // Name the grid "Map".
    grid_levelset->setName("Map");

    // Save new grid
    openvdb::io::File("pcdgrid.vdb").write({grid_levelset});
}