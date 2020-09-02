#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <random>

template<typename T>
bool get(const ros::NodeHandle &nh, const std::string &key, T &val)
{
  if (!nh.getParam(key, val))
  {
    ROS_ERROR_STREAM("Failed to get '" << key << "' parameter");
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "colorized_mesh_display_test_node");
  ros::NodeHandle nh, pnh("~");

  std::string path;
  if(!get(pnh, "mesh_file", path))
    return -1;

  std::string base_frame;
  if(!get(pnh, "base_frame", base_frame))
    return -1;

  // Load the mesh into a ROS message
  pcl_msgs::PolygonMesh msg;
  {
    pcl::PolygonMesh mesh;
    if(pcl::io::loadPolygonFile(path, mesh) < 0)
    {
      ROS_ERROR_STREAM("Failed to load mesh file from '" << path << "'");
      return -1;
    }
    ROS_INFO_STREAM("Successfully loaded mesh file");

    // Convert the mesh
    pcl_conversions::fromPCL(mesh, msg);
    msg.header.frame_id = base_frame;
    msg.header.stamp = ros::Time::now();
  }

  // Publish the mesh
  ros::Publisher pub = nh.advertise<pcl_msgs::PolygonMesh>("colorized_mesh", 1, true);
  pub.publish(msg);
  ROS_INFO_STREAM("Published colorized mesh file");

  ros::spin();

  return 0;
}
