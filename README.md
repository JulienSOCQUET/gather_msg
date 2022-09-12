# gather_msg
This ROS package is used to gather ROS messages in coherent structures (eg. gather PointCloud2 msgs and write them as a pcd file).

## gather_pcl_msg
This node is used to gather PointCloud2 messages from a given topic and save the final point cloud to a designated path. This node listens to the topic when it starts and terminates when user input CTRL-C.
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
In generally, the minium format of the command line is:
```bash
rosrun gather_msg gather_pcl_msg /velodyne_points
```
Listen to /velodyne_points and save to $PWD directory with the timestamp relative filename.

Advanced, you can use '-p' or '-o' to assign other folder to save.
'-t' argument decide frame count interval to divide into seperate partition file. This would be processed in other thread, don't worry about blocking main thread.

## gather_pcl_msg_bag
This executable is used to gather PointCloud2 or LaserScan messages from a given topic of a given bag file using a given trajectory (set of poses) and save the final point cloud to a designated path.
```bash
rosrun gather_msg gather_pcl_msg_bag  --help
Usage: gather_pcl_msg_bag BAGFILE [options]

Create a point cloud from a bag file containing scan or point cloud data.

Allowed options:
  --help                       print help message
  --output OUTPUT              explictly output file name
  --poses POSES                poses used to gather the clouds
  --static-transform TRANSFORM (=X Y Z QX QY QZ QW or =X Y Z ROLL PITCH YAW) 
                               static transform used to gather the clouds, 
                               applied before the poses
  --topic TOPIC                topic to read the cloud msgs

```
In generally, the minium format of the command line is:
```bash
rosrun gather_msg gather_pcl_msg_bag bagfile.bag
```
Read the bag file, find the topic with PointCloud2 or LaserScan type, gather the points without using any transform and save them to the $PWD directory with the current time as filename.

The trajectory file should provide the transforms as time tx ty tz qw qx qy qz (in this specific order on one line) at a frequency of 100Hz (for now). See the source for all the assumptions.

## TODO
* Enable gather_pcl_msg and gather_pcl_msg_bag to gather LiDAR packets messages besides PointCloud2 messages.
* Enable gather_pcl_msg and gather_pcl_msg_bag to write other point cloud formats (PLY, LAS, LAZ, E57).
* See gather_pcl_msg_bag source code for other possible improvements.