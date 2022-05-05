#include <iostream>
#include <fstream>
#include <signal.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <stdlib.h>
#include <limits>

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace po = boost::program_options;

// Get the next transform from the text trajectory
bool getNextTransform(std::ifstream& traj, geometry_msgs::TransformStamped& transform) {
  std::string pose;
  if(!std::getline(traj, pose))
    return false;
  // Assumes that we have: time tx ty tz qw qx qy qz
  std::stringstream ssPose (pose);
  std::string item;
  std::getline(ssPose, item, ' ');
  transform.header.stamp = ros::Time(std::stod(item));
  std::getline(ssPose, item, ' ');
  transform.transform.translation.x = std::stod(item);
  std::getline(ssPose, item, ' ');
  transform.transform.translation.y = std::stod(item);
  std::getline(ssPose, item, ' ');
  transform.transform.translation.z = std::stod(item);
  std::getline(ssPose, item, ' ');
  transform.transform.rotation.w = std::stod(item);
  std::getline(ssPose, item, ' ');
  transform.transform.rotation.x = std::stod(item);
  std::getline(ssPose, item, ' ');
  transform.transform.rotation.y = std::stod(item);
  std::getline(ssPose, item, ' ');
  transform.transform.rotation.z = std::stod(item);
  return true;
}

// Merge and export frames to pcd file
void mergeAndExport(std::string bagFile, std::string topic, std::string trajFile, std::string outputFile)
{
  std::cout << "Merge and export." << std::endl; 
  pcl::PCLPointCloud2 cloudOut;
  double deltaStamp;
  int count = 0;
  
  geometry_msgs::TransformStamped transform;
  std::ifstream traj (trajFile);
  if (!traj.is_open())
    return;
  traj.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  getNextTransform(traj, transform);
  
  rosbag::Bag bag(bagFile);
  rosbag::View view(bag, rosbag::TopicQuery(topic));
  BOOST_FOREACH(rosbag::MessageInstance const pclMsg, view)
    {
      sensor_msgs::PointCloud2::ConstPtr cloudPtr = pclMsg.instantiate<sensor_msgs::PointCloud2>();
      if (cloudPtr != NULL) {
	deltaStamp = (cloudPtr->header.stamp + 0.05 - transform.header.stamp).toSec(); // 0.05s to take the middle time of the frame
	while(deltaStamp > 0.005) {
	  if(!getNextTransform(traj, transform))
	    break;
	  deltaStamp = (cloudPtr->header.stamp - transform.header.stamp).toSec();
	}

	if(abs(deltaStamp) < 0.005) {
	  sensor_msgs::PointCloud2 cloudTf;
	  tf2::doTransform(*cloudPtr, cloudTf, transform);
	  pcl::PCLPointCloud2 cloudPcl;
	  pcl_conversions::moveToPCL(cloudTf, cloudPcl);
	  cloudOut += cloudPcl;
	  std::cout << "Frame added: " << ++count << std::endl;
	} else {
	  std::cout << "Warning: frame skipped." << std::endl;
	}
      }
    }
  bag.close();
  traj.close();

  std::cout << "Merged " << cloudOut.width * cloudOut.height << " points." << std::endl;
  if ((cloudOut.width * cloudOut.height) == 0) {
    std::cout << "Point cloud empty, exit." << std::endl;
    return;
  }
  std::cout << "Start saving, please wait...\n" << outputFile << std::endl;
  pcl::PCDWriter writer;
  writer.writeBinary(outputFile, cloudOut);
  std::cout << "File saved." << std::endl;
}

int main(int argc, char** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "print help message")("output,o", po::value<std::string>(), "explictly output file name")("trajectory,t", po::value<std::string>(), "trajectory used to gather the clouds")("topic", po::value<std::string>(), "topic to read the cloud msgs")("bagfile,b", "topic which would be subscribed");
  po::positional_options_description p;
  p.add("bagfile", -1);
  po::variables_map vm;

  std::string outputFile, trajFile, topic, bagFile;

  try {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);
  } catch (std::exception& e) {
    throw ros::Exception(e.what());
  }
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit(0);
  }
  if (vm.count("output")) {
    outputFile = vm["output"].as<std::string>();
  } else {
    std::cout << desc << std::endl;
    exit(0);
  }
  if (vm.count("trajectory")) {
    trajFile = vm["trajectory"].as<std::string>();
  } else {
    std::cout << desc << std::endl;
    exit(0);
  }
  if (vm.count("topic")) {
    topic = vm["topic"].as<std::string>();
  } else {
    std::cout << desc << std::endl;
    exit(0);
  }
  if (vm.count("bagfile")) {
    bagFile = vm["bagfile"].as<std::string>();
  } else {
    std::cout << desc << std::endl;
    exit(0);
  }

  ROS_INFO_STREAM("|Output:\t| "       << outputFile);
  ROS_INFO_STREAM("|Trajectory:\t| " << trajFile);
  ROS_INFO_STREAM("|Topic:\t\t| "      << topic);
  ROS_INFO_STREAM("|Bagfile:\t| "    << bagFile);

  mergeAndExport(bagFile, topic, trajFile, outputFile); 
  
  return 0;
}
