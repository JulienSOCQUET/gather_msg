#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <limits>
#include <vector>
#include <math.h>

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointField.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace po = boost::program_options;

constexpr float Pi {3.14159265358979323846};


////////////////////////////////////////////////////////////////////////////////////////////////////



// Structure to store a 2D LiDAR calibration
struct Calibration
{
  float offset;
  float eccentricity;

  Calibration(const float& off = 0, const float& ecc = 0)
    : offset(off),
      eccentricity(ecc)
  {
  }

  Calibration(const std::vector<float>& calibration)
  {
    if (calibration.size() == 2)
    {
      offset       = calibration[0];
      eccentricity = calibration[1];
    }
    else
    {
      throw std::runtime_error("The calibration vector should contain 2 elements.");
    }
  }
};

inline bool operator==(const Calibration& c1, const Calibration& c2)
{
  return c1.offset == c2.offset && c1.eccentricity == c2.eccentricity;
}

inline bool operator!=(const Calibration& c1, const Calibration& c2)
{
  return !(c1 == c2);
}



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



// Interpolate two transforms
Eigen::Affine3d interpolate(const geometry_msgs::TransformStamped& ts1, const geometry_msgs::TransformStamped& ts2, const double ratio)
{
  double complement = 1 - ratio;
  Eigen::Translation3d t(ts1.transform.translation.x * complement + ts2.transform.translation.x * ratio,
                         ts1.transform.translation.y * complement + ts2.transform.translation.y * ratio,
                         ts1.transform.translation.z * complement + ts2.transform.translation.z * ratio);
  Eigen::Quaterniond q1(ts1.transform.rotation.w, ts1.transform.rotation.x, ts1.transform.rotation.y, ts1.transform.rotation.z);
  Eigen::Quaterniond q2(ts2.transform.rotation.w, ts2.transform.rotation.x, ts2.transform.rotation.y, ts2.transform.rotation.z);
  return Eigen::Affine3d(t * q1.slerp(ratio, q2));
}




////////////////////////////////////////////////////////////////////////////////////////////////////



// Convert a sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
void toPCL(const sensor_msgs::PointCloud2& cloud, pcl::PCLPointCloud2& cloudPcl, Calibration)
{
  pcl_conversions::toPCL(cloud, cloudPcl);
}

// Class to convert a sensor_msgs::LaserScan to ???
class LaserProjection
{
public:
  void projectLaser(const sensor_msgs::LaserScan& scanIn, sensor_msgs::PointCloud2& cloudOut, const Calibration& calibration)
  {
    size_t scanSize = scanIn.ranges.size();
    Eigen::ArrayXf ranges(scanSize);
    Eigen::ArrayXXf outputXY(scanSize, 2);
    float outputZ = 0;

    // Get the ranges into Eigen format
    for (size_t i = 0; i < scanSize; ++i)
    {
      ranges(i) = scanIn.ranges[i];
    }

    // Check if our existing XYLut is valid
    if (m_XYLut.rows() != scanSize || m_AngleMin != scanIn.angle_min || m_AngleMax != scanIn.angle_max
        || m_AngleIncrement != scanIn.angle_increment || m_Calibration != calibration)
    {
      ROS_INFO("[projectLaser] No precomputed lookup table given. Computing one.");
      m_XYLut          = Eigen::ArrayXXf (scanSize, 2);
      m_AngleMin       = scanIn.angle_min;
      m_AngleMax       = scanIn.angle_max;
      m_AngleIncrement = scanIn.angle_increment;
      m_Calibration    = calibration;
      // Spherical->Cartesian projection
      for (size_t i = 0; i < scanSize; ++i)
      {
        float angle   = scanIn.angle_min + i * scanIn.angle_increment;
        angle         = angle + calibration.offset + calibration.eccentricity * sin(angle + Pi / 2);
        m_XYLut(i, 0) = cos(angle);
        m_XYLut(i, 1) = sin(angle);
      }
    }

    outputXY  = m_XYLut.colwise() * ranges;

    // Set fields of the output cloud
    cloudOut.header = scanIn.header;
    cloudOut.height = 1;
    cloudOut.width  = scanIn.ranges.size ();
    cloudOut.fields.resize (7);
    cloudOut.fields[0].name = "x";
    cloudOut.fields[0].offset = 0;
    cloudOut.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloudOut.fields[0].count = 1;
    cloudOut.fields[1].name = "y";
    cloudOut.fields[1].offset = 4;
    cloudOut.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloudOut.fields[1].count = 1;
    cloudOut.fields[2].name = "z";
    cloudOut.fields[2].offset = 8;
    cloudOut.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloudOut.fields[2].count = 1;
    cloudOut.fields[3].name = "intensity";
    cloudOut.fields[3].offset = 12;
    cloudOut.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloudOut.fields[3].count = 1;
    cloudOut.fields[4].name = "t";
    cloudOut.fields[4].offset = 16;
    cloudOut.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
    cloudOut.fields[4].count = 1;
    cloudOut.fields[5].name = "timestamp";
    cloudOut.fields[5].offset = 20;
    cloudOut.fields[5].datatype = sensor_msgs::PointField::FLOAT64;
    cloudOut.fields[5].count = 1;
    cloudOut.fields[6].name = "range";
    cloudOut.fields[6].offset = 28;
    cloudOut.fields[6].datatype = sensor_msgs::PointField::FLOAT32;
    cloudOut.fields[6].count = 1;

    cloudOut.point_step = 32;
    cloudOut.row_step   = cloudOut.point_step * cloudOut.width;
    cloudOut.data.resize(cloudOut.row_step * cloudOut.height);
    cloudOut.is_dense   = false;

    unsigned int count = 0;
    for (size_t i = 0; i < scanSize; ++i)
    {
      // Check to see if we want to keep the point
      const float range = scanIn.ranges[i];
      if (range < scanIn.range_max && range >= scanIn.range_min)
      {
        // Copy XYZ
        memcpy(&cloudOut.data[count * cloudOut.point_step + cloudOut.fields[0].offset], &outputXY(i, 0), sizeof(float));
        memcpy(&cloudOut.data[count * cloudOut.point_step + cloudOut.fields[1].offset], &outputXY(i, 1), sizeof(float));
        memcpy(&cloudOut.data[count * cloudOut.point_step + cloudOut.fields[2].offset], &outputZ, sizeof(float));

        // Copy intensity
        memcpy(&cloudOut.data[count * cloudOut.point_step + cloudOut.fields[3].offset], &scanIn.intensities[i], sizeof(float));

        //Copy time
        float time = i * scanIn.time_increment;
        memcpy(&cloudOut.data[count * cloudOut.point_step + cloudOut.fields[4].offset], &time, sizeof(float));
        double timestamp = time + scanIn.header.stamp.toSec();
        memcpy(&cloudOut.data[count * cloudOut.point_step + cloudOut.fields[5].offset], &timestamp, sizeof(double));

        // Copy range
        memcpy(&cloudOut.data[count * cloudOut.point_step + cloudOut.fields[6].offset], &range, sizeof(float));

        ++count;
      }
    }

    // Resize if necessary
    cloudOut.width = count;
    cloudOut.row_step   = cloudOut.point_step * cloudOut.width;
    cloudOut.data.resize(cloudOut.row_step   * cloudOut.height);
  }

private:
  float m_AngleMin;
  float m_AngleMax;
  float m_AngleIncrement;
  Calibration m_Calibration;
  Eigen::ArrayXXf m_XYLut;
};


LaserProjection laserProjection;

// Convert a sensor_msgs::LaserScan to pcl::PCLPointCloud2
void toPCL(const sensor_msgs::LaserScan& scan, pcl::PCLPointCloud2& cloudPcl, const Calibration& calibration)
{
  sensor_msgs::PointCloud2 cloud;
  laserProjection.projectLaser(scan, cloud, calibration);
  pcl_conversions::toPCL(cloud, cloudPcl);
}




////////////////////////////////////////////////////////////////////////////////////////////////////



float getTimePoint(const pcl::PCLPointCloud2& cloud, const int& pointIdx, const int& timeIdx)
{
  if (cloud.fields[timeIdx].datatype == sensor_msgs::PointField::UINT32)
  {
    uint32_t ts;
    memcpy(&ts, &cloud.data[pointIdx + cloud.fields[timeIdx].offset], sizeof(uint32_t));
    return (ts * 1e-9);
  }
  else if (cloud.fields[timeIdx].datatype == sensor_msgs::PointField::FLOAT32)
  {
    float ts;
    memcpy(&ts, &cloud.data[pointIdx + cloud.fields[timeIdx].offset], sizeof(float));
    return ts;
  }
  else
  {
    throw std::runtime_error("Time field type not supported.");
  }
}


  
////////////////////////////////////////////////////////////////////////////////////////////////////



// Merge, transform and export frames to pcd file
template <typename MessageT>
void mergeAndExport(const rosbag::Bag& bag, const std::string& topic, const std::string& trajFile,
                    const std::string& outputFile, const Eigen::Affine3d& staticTransform, const Calibration& calibration)
{
  std::cout << "Merge and export." << std::endl;
  pcl::PCLPointCloud2 cloudOut;
  double deltaStamp;
  int frameCount = 0;

  // Get the first transform
  geometry_msgs::TransformStamped transformStamped;
  std::ifstream traj (trajFile);
  if (!trajFile.empty())
  {
    if (!traj.is_open())
    {
      std::cerr << "Failed to open the pose file (trajectory)." << std::endl;
      return;
    }
    traj.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Skip the first line
    getNextTransform(traj, transformStamped);
  }
  geometry_msgs::TransformStamped lastTransformStamped = transformStamped;

  // Read the messages from the given topic
  rosbag::View view(bag, rosbag::TopicQuery(topic));
  BOOST_FOREACH(rosbag::MessageInstance const pclMsg, view)
  {
    typename MessageT::ConstPtr cloudPtr = pclMsg.instantiate<MessageT>();
    if (cloudPtr != NULL)
    {
      pcl::PCLPointCloud2 cloudPcl;
      toPCL(*cloudPtr, cloudPcl, calibration);

      // Transform the points if trajectory available
      if (!trajFile.empty())
      {
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
        for (int col = 0; col < cloudPcl.width; ++col)
        {
          // Assumes that the point cloud is organized (columns ordered by ascending timestamp)
          int point_offset = col * cloudPcl.point_step;
          float ts = getTimePoint(cloudPcl, point_offset, t_idx);

          // Get the transforms just before and just after the point
          deltaStamp = (cloudPtr->header.stamp - transformStamped.header.stamp).toSec() + ts;
          while(deltaStamp > 0)
          {
            lastTransformStamped = transformStamped;
            if(!getNextTransform(traj, transformStamped))
              break;
            deltaStamp = (cloudPtr->header.stamp - transformStamped.header.stamp).toSec() + ts;
          }
          
          // Interpolate transforms and apply transform
          // Assumes that the points from a column have the same timestamp
          double deltaTransformStamp = (transformStamped.header.stamp - lastTransformStamped.header.stamp).toSec();
          if (deltaTransformStamp != 0)
          {
            Eigen::Affine3d transform;
            double ratio = deltaStamp / deltaTransformStamp;
            transform = interpolate(transformStamped, lastTransformStamped, 1 - ratio);
            transform = transform * staticTransform;
            for (int row = 0; row < cloudPcl.height; ++row)
            {
              Eigen::Vector3d pt(*reinterpret_cast<const float *>(&cloudPcl.data[point_offset + cloudPcl.fields[x_idx].offset]),
                                 *reinterpret_cast<const float *>(&cloudPcl.data[point_offset + cloudPcl.fields[y_idx].offset]),
                                 *reinterpret_cast<const float *>(&cloudPcl.data[point_offset + cloudPcl.fields[z_idx].offset]));
              Eigen::Vector3f ptOut = (transform * pt).cast <float> ();

              memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[x_idx].offset], &ptOut[0], sizeof(float));
              memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[y_idx].offset], &ptOut[1], sizeof(float));
              memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[z_idx].offset], &ptOut[2], sizeof(float));
              point_offset += cloudPcl.row_step;
            }
          }
          // Set point as NAN if no interpolation
          else
          {
            Eigen::Vector3f ptOut(std::numeric_limits<float>::quiet_NaN(),
                                  std::numeric_limits<float>::quiet_NaN(),
                                  std::numeric_limits<float>::quiet_NaN());
            for (int row = 0; row < cloudPcl.height; ++row) {
              memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[x_idx].offset], &ptOut[0], sizeof(float));
              memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[y_idx].offset], &ptOut[1], sizeof(float));
              memcpy(&cloudPcl.data[point_offset + cloudPcl.fields[z_idx].offset], &ptOut[2], sizeof(float));

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



// Convert a vector storing a transform to an Eigen::Affine3d
Eigen::Affine3d toAffine (const std::vector<float>& transform)
{
  Eigen::Quaterniond q;
  Eigen::Translation3d t(transform[0], transform[1], transform[2]);
  if (transform.size() == 7)
  {
    q = Eigen::Quaterniond(transform[6], transform[3], transform[4], transform[5]); // w x y z
  }
  else if (transform.size() == 6)
  {
    q = Eigen::AngleAxisd(transform[3], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(transform[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(transform[5], Eigen::Vector3d::UnitZ());
  }
  else
  {
    throw std::runtime_error("The transfrom vector should contain 6 or 7 elements.");
  }
  return Eigen::Affine3d(t * q);
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



// Get current time as string
void printHelp(const std::string& name, const po::options_description& options)
{
  std::cout << "Usage: " << name << " BAGFILE [options]\n\n";
  std::cout << "Create a point cloud from a bag file containing scan or point cloud data.\n\n";
  std::cout << options << std::endl;
}



////////////////////////////////////////////////////////////////////////////////////////////////////



int main(int argc, char** argv)
{  
  po::options_description visibleDesc("Allowed options");
  visibleDesc.add_options()
    ("help", "print help message")
    ("output", po::value<std::string>()->value_name("OUTPUT"), "explictly output file name")
    ("trajectory", po::value<std::string>()->value_name("TRAJECTORY"), "trajectory used to gather the clouds")
    ("static-transform", po::value<std::vector<float> >()->multitoken()->value_name("TRANSFORM"), "(=X Y Z QX QY QZ QW or =X Y Z ROLL PITCH YAW) static transform used to gather the clouds, applied before the trajectory")
    ("calibration", po::value<std::vector<float> >()->multitoken()->value_name("CALIBRATION"), "(=Offset Eccentricity) encoder calibration for 2D LiDARs")
    ("topic", po::value<std::string>()->value_name("TOPIC"), "topic to read the cloud msgs");
  po::options_description desc;
  desc.add_options()
    ("bagfile", "bag file to read the cloud msgs");
  desc.add(visibleDesc);
  po::positional_options_description p;
  p.add("bagfile", -1);
  po::variables_map vm;

  std::string outputFile, trajFile, topic, bagFile;
  std::vector<float> staticTransform, calibration;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).positional(p).run(), vm);
    po::notify(vm);
  } catch (std::exception& e) {
    throw ros::Exception(e.what());
  }
  
  if (vm.count("help")) {
    printHelp(argv[0], visibleDesc);
    exit(0);
  }
  if (vm.count("output")) {
    outputFile = vm["output"].as<std::string>();
  } else {
    outputFile = getDefaultOutput();
  }
  if (vm.count("trajectory")) {
    trajFile = vm["trajectory"].as<std::string>();
  }
  if (vm.count("static-transform")) {
    staticTransform = vm["static-transform"].as<std::vector<float> >();
    if (staticTransform.size() != 6 && staticTransform.size() != 7) {
      printHelp(argv[0], visibleDesc);
      exit(0);
    }
  } else {
    staticTransform = std::vector<float>({0, 0, 0, 0, 0, 0, 1});
  }
  if (vm.count("calibration")) {
    calibration = vm["calibration"].as<std::vector<float> >();
    if (calibration.size() != 2) {
      printHelp(argv[0], visibleDesc);
      exit(0);
    }
  } else {
    calibration = std::vector<float>({0, 0});
  }
  if (vm.count("topic")) {
    topic = vm["topic"].as<std::string>();
  }
  if (vm.count("bagfile")) {
    bagFile = vm["bagfile"].as<std::string>();
  } else {
    printHelp(argv[0], visibleDesc);
    exit(0);
  }

  // Open bag file
  rosbag::Bag bag(bagFile);
  if (!bag.isOpen())
  {
    std::cerr << "Failed to open the bag file." << std::endl;
    return 1;
  }

  // Find topic type, and find appropriate topic if not provided
  std::string type;
  std::vector<std::string> types;
  types.push_back("sensor_msgs/PointCloud2");
  types.push_back("sensor_msgs/LaserScan");
  rosbag::View view(bag, rosbag::TypeQuery(types));
  std::vector<const rosbag::ConnectionInfo *> connectionInfos = view.getConnections();
  BOOST_FOREACH(const rosbag::ConnectionInfo *info, connectionInfos)
  {
    if (info->topic == topic)
    {
      type = info->datatype;
      break;
    }
    else if (topic.empty())
    {
      topic = info->topic;
      type  = info->datatype;
      break;
    }
  }

  if (topic.empty())
  {
    ROS_WARN("No topic found.");
  }

  ROS_INFO_STREAM("|Output:\t| "  << outputFile);
  ROS_INFO_STREAM("|Trajectory:\t| " << trajFile);
  ROS_INFO_STREAM("|Topic:\t\t| " << topic);
  ROS_INFO_STREAM("|Bagfile:\t| " << bagFile);

  if (type == "sensor_msgs/PointCloud2")
  {
    mergeAndExport<sensor_msgs::PointCloud2>(bag, topic, trajFile, outputFile, toAffine(staticTransform), Calibration(calibration));
  }
  else if (type == "sensor_msgs/LaserScan")
  {
    mergeAndExport<sensor_msgs::LaserScan>(bag, topic, trajFile, outputFile, toAffine(staticTransform), Calibration(calibration));
  }
  else
  {
    ROS_ERROR("Wrong topic type.");
  }
    
  // Close bag file
  bag.close();

  return 0;
}
