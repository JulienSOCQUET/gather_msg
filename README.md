# gather_msg
This ROS package is used to gather ROS messages in coherent structures (eg. gather PointCloud2 msgs and write them as a pcd file).

## gather_pcl_msg
This node is used to gather PCL point clouds messages from a given topic and save the final point cloud to a designated path. This node listens to the topic when it starts and terminates when user input CTRL-C.
If the cloud data would surpass your memory capacity. Partition feature is available.
You can read help info by
```bash
rosrun gather_msg gather_pcl_msg  -h
Allowed options:
  -h [ --help ]               print help message
  -o [ --output ] arg         explictly output file name
  -b [ --binary ] arg (=1)    write binary PCD
  -p [ --prefix ] arg         output prefix
  -t [ --partition ] arg (=0) frame num of a partition ( 0 for no parititon)
  --topic                     topic which would be subscribed
```
In generally, the minium format of the command line like this:
```bash
rosrun gather_msg gather_pcl_msg /velodyne_points
```
Listen to /velodyne_points and save to $PWD directory with the timestamp relative filename.

Advanced, you can use '-p' or '-o' to assign other folder to save.
'-t' argument decide frame count interval to divide into seperate partition file. This would be processed in other thread, don't worry about blocking main thread.

## gather_pcl_msg_ctslam
This executable is used to gather PCL point clouds messages from a given topic of a given bag file using a given trajectory (set of poses) and save the final point cloud to a designated path.
```bash
rosrun gather_msg gather_pcl_msg_ctslam  -h
Allowed options:
  -h [ --help ]         print help message
  -o [ --output ] arg   explictly output file name
  -p [ --poses ] arg    poses used to gather the clouds
  -t [ --topic ] arg    topic to read the cloud msgs
  --bagfile             bag file to read the cloud msgs
```
For now it only supports Ouster LiDARs. The trajectory file should provide the transforms as time tx ty tz qw qx qy qz (in this specific order on one line) at a frequency of 100Hz (for now). See the source for all the assumptions.

## TODO
* Enable gather_pcl_msg and gather_pcl_msg_ctslam to gather LiDAR packets messages besides PointCloud2 messages.
* Enable gather_pcl_msg and gather_pcl_msg_ctslam to write other point cloud formats (PLY, LAS, LAZ, E57).
* See gather_pcl_msg_ctslam source code for other possible improvements.