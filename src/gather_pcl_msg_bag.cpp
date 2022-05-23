#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <limits>

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace po = boost::program_options;



////////////////////////////////////////////////////////////////////////////////////////////////////



// Get the next transform from the text trajectory
bool getNextTransform(std::ifstream& traj, geometry_msgs::TransformStamped& transformStamped)
{
  std::string pose;
  if(!std::getline(traj, pose))
    return false;
  // Assumes that we have: time tx ty tz qw qx qy qz
  std::stringstream ssPose (pose);
  std::string item;
  std::getline(ssPose, item, ' ');
  transformStamped.header.stamp = ros::Time(std::stod(item));
  std::getline(ssPose, item, ' ');
  transformStamped.transform.translation.x = std::stod(item);
  std::getline(ssPose, item, ' ');
  transformStamped.transform.translation.y = std::stod(item);
  std::getline(ssPose, item, ' ');
  transformStamped.transform.translation.z = std::stod(item);
  std::getline(ssPose, item, ' ');
  transformStamped.transform.rotation.w = std::stod(item);
  std::getline(ssPose, item, ' ');
  transformStamped.transform.rotation.x = std::stod(item);
  std::getline(ssPose, item, ' ');
  transformStamped.transform.rotation.y = std::stod(item);
  std::getline(ssPose, item, ' ');
  transformStamped.transform.rotation.z = std::stod(item);
  return true;
}



////////////////////////////////////////////////////////////////////////////////////////////////////



// Merge, transform and export frames to pcd file
void mergeAndExport(rosbag::Bag& bag, std::string topic, std::string trajFile, std::string outputFile)
{
  std::cout << "Merge and export." << std::endl;
  pcl::PCLPointCloud2 cloudOut;
  double deltaStamp;
  int frameCount = 0;

  // Get the first transform
  geometry_msgs::TransformStamped transformStamped;
  std::ifstream traj (trajFile);
  if (!trajFile.empty()) {
    if (!traj.is_open()) {
      std::cerr << "Failed to open the pose file (trajectory)." << std::endl;
      return;
    }
    traj.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Skip the first line
    getNextTransform(traj, transformStamped);
  }

  // Read the messages from the given topic
  rosbag::View view(bag, rosbag::TopicQuery(topic));
  BOOST_FOREACH(rosbag::MessageInstance const pclMsg, view) {
    sensor_msgs::PointCloud2::ConstPtr cloudPtr = pclMsg.instantiate<sensor_msgs::PointCloud2>();
    if (cloudPtr != NULL) {
      pcl::PCLPointCloud2 cloudPcl;
      pcl_conversions::toPCL(*cloudPtr, cloudPcl);

      // Transform the points if trajectory available
      if (!trajFile.empty()) {
	// Get the index of the fields
	int x_idx = pcl::getFieldIndex (cloudPcl, "x");
	int y_idx = pcl::getFieldIndex (cloudPcl, "y");
	int z_idx = pcl::getFieldIndex (cloudPcl, "z");
	int t_idx = std::max(pcl::getFieldIndex (cloudPcl, "t"), pcl::getFieldIndex (cloudPcl, "time"));
	if (x_idx == -1 || y_idx == -1 || z_idx == -1 || t_idx == -1) {
	  std::cerr << "The points need to have x, y, z and t/time attributes." << std::endl;
	  return;
	}

	// Go over all points
	uint32_t ts; // Should be the time offset in nano seconds from the beginning of the sweep
	int point_offset;
	for (int col = 0; col < cloudPcl.width; ++col) {
	  // Assumes that the point cloud is organized (columns ordered by ascending timestamp)
	  // Get the local timestamp
	  point_offset = col * cloudPcl.point_step;
	  memcpy (&ts, &cloudPcl.data[point_offset + cloudPcl.fields[t_idx].offset], sizeof(uint32_t));

	  // Get a transform close enough in time
	  deltaStamp = (cloudPtr->header.stamp - transformStamped.header.stamp).toSec() + 1e-9 * ts;
	  while(deltaStamp > 0.005) { // Assumes a trajectory sampling at 100Hz
	    if(!getNextTransform(traj, transformStamped))
	      break;
	    deltaStamp = (cloudPtr->header.stamp - transformStamped.header.stamp).toSec() + 1e-9 * ts;
	  }

	  // Apply transform if time close enough
	  if(abs(deltaStamp) < 0.005) { // Assumes a trajectory sampling at 100Hz
	    for (int row = 0; row < cloudPcl.height; ++row) {
	      // Assumes that the points from a column have the same timestamp
	      Eigen::Affine3d transform = tf2::transformToEigen(transformStamped); // TODO improve by interpolating transforms

	      Eigen::Vector3d pt(*reinterpret_cast<const float *>(&cloudPcl.data[point_offset + cloudPcl.fields[x_idx].offset]),
				 *reinterpret_cast<const float *>(&cloudPcl.data[point_offset + cloudPcl.fields[y_idx].offset]),
				 *reinterpret_cast<const float *>(&cloudPcl.data[point_offset + cloudPcl.fields[z_idx].offset]));
	      Eigen::Vector3f pt_out = (transform * pt).cast <float> ();

	      memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[x_idx].offset], &pt_out[0], sizeof(float));
	      memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[y_idx].offset], &pt_out[1], sizeof(float));
	      memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[z_idx].offset], &pt_out[2], sizeof(float));
	      point_offset += cloudPcl.row_step;
	    }
	    // Set point as NAN if transform time too far
	  } else {
	    Eigen::Vector3f pt_out(std::numeric_limits<float>::quiet_NaN(),
				   std::numeric_limits<float>::quiet_NaN(),
				   std::numeric_limits<float>::quiet_NaN());
	    for (int row = 0; row < cloudPcl.height; ++row) {
	      memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[x_idx].offset], &pt_out[0], sizeof(float));
	      memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[y_idx].offset], &pt_out[1], sizeof(float));
	      memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[z_idx].offset], &pt_out[2], sizeof(float));

	      point_offset += cloudPcl.row_step;
	    }
	    std::cout << "Warning: column skipped." << std::endl;
	  }
	}
      }
      cloudOut += cloudPcl;
      std::cout << "Frame added: " << ++frameCount << std::endl;
    }
  }
  traj.close();

  // Write point cloud if not empty
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



////////////////////////////////////////////////////////////////////////////////////////////////////



// Get current time as string
std::string getDefaultOutput()
{
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet* const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(), f));
    msg << now << ".pcd";
    return msg.str();
}



////////////////////////////////////////////////////////////////////////////////////////////////////



int main(int argc, char** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "print help message")("output,o", po::value<std::string>(), "explictly output file name")("poses,p", po::value<std::string>(), "poses used to gather the clouds")("topic,t", po::value<std::string>(), "topic to read the cloud msgs")("bagfile", "bag file to read the cloud msgs");
  po::positional_options_description p;
  p.add("bagfile", -1);
  po::variables_map vm;

  std::string outputFile, poseFile, topic, bagFile;

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
    outputFile = getDefaultOutput();
  }
  if (vm.count("poses")) {
    poseFile = vm["poses"].as<std::string>();
  }
  if (vm.count("topic")) {
    topic = vm["topic"].as<std::string>();
  }
  if (vm.count("bagfile")) {
    bagFile = vm["bagfile"].as<std::string>();
  } else {
    std::cout << desc << std::endl;
    exit(0);
  }

  // Open bag file
  rosbag::Bag bag(bagFile);
  if (!bag.isOpen()) {
    std::cerr << "Failed to open the bag file." << std::endl;
    return 1;
  }

  // Find appropriate topic if not provided
  if (topic.empty()) {
    rosbag::View view(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    std::vector<const rosbag::ConnectionInfo *> connectionInfos = view.getConnections();
    BOOST_FOREACH(const rosbag::ConnectionInfo *info, connectionInfos) {
      if (topic.empty())
	topic = info->topic;
      else if(topic != info->topic) {
	std::cerr << "Several topics with PointCloud2 type found, please specify the one to be used." << std::endl;
	return 1;
      }
    }
  }

  ROS_INFO_STREAM("|Output:\t| "  << outputFile);
  ROS_INFO_STREAM("|Poses:\t\t| " << poseFile);
  ROS_INFO_STREAM("|Topic:\t\t| " << topic);
  ROS_INFO_STREAM("|Bagfile:\t| " << bagFile);

  mergeAndExport(bag, topic, poseFile, outputFile);

  // Close bag file
  bag.close();

  return 0;
}
