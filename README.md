# VDB Mapping

## Installation 
1. Install vdb_mapping dependencies:
   ```sh
   apt-get install -y libeigen3-dev
   apt-get install -y libtbb-dev
   apt-get install -y libpcl-dev
   apt-get install -y libilmbase-dev
   ```


## Run
1. Run the mapping node:
   ```sh
   ros2 bag play 1005_07_img10hz600p --clock
   ```
2. Launch the ros2bag:
   ```sh
   ros2 launch vdb_mapping_ros2 vdb_mapping_ros2.launch.py
   ```
3. Save the map:
   ```sh
   ros2 service call /vdb_mapping/save_map std_srvs/srv/Trigger "{}" 
   ```
4. After mapping use the script in dropbox to transform to level set narrow band:


## Check the map
1. Print information on terminal:
   ```sh
   vdb_print -l pcdgrid.vdb
   ```
2. Use openVDB Viewer :
   ```sh
   ./OpenVDBViewer/OpenVDBViewer
   ```