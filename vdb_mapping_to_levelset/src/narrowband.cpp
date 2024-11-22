#include <openvdb/openvdb.h>
#include <openvdb/tools/TopologyToLevelSet.h>

int main()
{
    openvdb::initialize();

    // Create a VDB file object.
    openvdb::io::File file("map_1006_no_levelset.vdb");

    // Open the file.  This reads the file header, but not any grids.
    file.open();

    // Print the names of the grid
    for (openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter)
    {
        std::cout << "Grid name: " << nameIter.gridName() << std::endl;

    }

    // Retrieve a shared pointer
    openvdb::GridBase::Ptr baseGrid;
    baseGrid = file.readGrid("[0]");

    // Close the file
    file.close();

    // Cast the generic grid pointer to a FloatGrid pointer.
    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);

    // Transform to level set
    openvdb::FloatGrid::Ptr grid_levelset = nullptr;
    grid_levelset = openvdb::tools::topologyToLevelSet(*grid, 3, 1, 0, 0);

    // Save new grid
    openvdb::io::File("res_7cm_051_thres_max_levelset.vdb").write({grid_levelset});
}