#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <limits>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace po = boost::program_options;



////////////////////////////////////////////////////////////////////////////////////////////////////



// Merge, transform and export frames to pcd file
void analyse(rosbag::Bag& bag, std::string topic, std::string outputFile)
{
  std::cout << "Analyse scan." << std::endl;
  int frameCount = 0;
  std::vector<std::vector<uint32_t>> infoRanges;

  // Read the messages from the given topic
  rosbag::View view(bag, rosbag::TopicQuery(topic));
  BOOST_FOREACH(rosbag::MessageInstance const pclMsg, view) {
    sensor_msgs::PointCloud2::ConstPtr cloudPtr = pclMsg.instantiate<sensor_msgs::PointCloud2>();
    if (cloudPtr != NULL) {
      pcl::PCLPointCloud2 cloudPcl;
      pcl_conversions::toPCL(*cloudPtr, cloudPcl);

      // Resize data storage
      int nrPoints = cloudPcl.width * cloudPcl.height;
      if (infoRanges.size() == 0)
	infoRanges.resize(nrPoints, std::vector<uint32_t>{0, 0, 0, std::numeric_limits<uint32_t>::max(), 0});
      // Check if the frame size is coherent
      else if (infoRanges.size() != nrPoints) {
	std::cout << "Warning: the frame has a different size!" << std::endl;
	continue;
      }

      // Get the index of the fields
      int rIdx = pcl::getFieldIndex (cloudPcl, "range");
      if (rIdx == -1) {
	std::cerr << "The points need to have range attribute." << std::endl;
	return;
      }

      // Go over all points
      uint32_t range; // Range in mm
      int rOffset = cloudPcl.fields[rIdx].offset;
      for (int cp = 0; cp < nrPoints; ++cp, rOffset += cloudPcl.point_step) {
	// Assumes that the frames are organized in the same way
	// Get the range value
	memcpy (&range, &cloudPcl.data[rOffset], sizeof(uint32_t));

	// Discard NAN/INF/-INF values
	if (!std::isfinite(range))
	  continue;

	// Discard zero values
	if (range == 0)
	  continue;

	// Accumulate info
	infoRanges[cp][0]++;
	infoRanges[cp][1] += range;
	infoRanges[cp][2] += range*range;
	infoRanges[cp][3]  = std::min(range, infoRanges[cp][3]);
	infoRanges[cp][4]  = std::max(range, infoRanges[cp][4]);
      }
      std::cout << "Frame processed: " << ++frameCount << std::endl;
    }
  }
  
  // Write info if not empty
  if (infoRanges.size() == 0) {
    std::cout << "No information, exit." << std::endl;
    return;
  }
  std::cout << "Start saving, please wait...\n" << outputFile << std::endl;
  std::ofstream infoFile(outputFile);
  infoFile << "//np mean mean2 min max";
  for (std::vector<std::vector<uint32_t>>::iterator it = infoRanges.begin(); it != infoRanges.end(); ++it)
    if ((*it)[0] != 0)
      infoFile << "\n" << (*it)[0] << "," << (*it)[1] / (*it)[0] << "," << (*it)[2] / (*it)[0] << "," << (*it)[3] << "," << (*it)[4];
  infoFile.close();
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
  msg << now << ".txt";
  return msg.str();
}



////////////////////////////////////////////////////////////////////////////////////////////////////



int main(int argc, char** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "print help message")("output,o", po::value<std::string>(), "explictly output file name")("topic,t", po::value<std::string>(), "topic to read the cloud msgs")("bagfile", "bag file to read the cloud msgs");
  po::positional_options_description p;
  p.add("bagfile", -1);
  po::variables_map vm;

  std::string outputFile, topic, bagFile;

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
  ROS_INFO_STREAM("|Topic:\t\t| " << topic);
  ROS_INFO_STREAM("|Bagfile:\t| " << bagFile);

  analyse(bag, topic, outputFile);

  // Close bag file
  bag.close();

  return 0;
}
