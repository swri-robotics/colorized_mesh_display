#include <colorized_mesh_display/ColorizedMeshStamped.h>
#include <colorized_mesh_display/utils.h>
#include <pcl/io/ply_io.h>
#include <ros/ros.h>
#include <random>

float generateRandomFloat()
{
  return float(rand()) / float(INT_MAX);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "colorized_mesh_display_test_node");
  ros::NodeHandle nh, pnh("~");

  std::string path;
  if(!pnh.getParam("ply_file", path))
  {
    ROS_ERROR("Failed to get 'ply_file' parameter");
    return -1;
  }

  std::string base_frame;
  if(!pnh.getParam("base_frame", base_frame))
  {
    ROS_ERROR("Failed to get 'base_frame' parameter");
    return -1;
  }

  unsigned int seed = 0;
//  pnh.param<unsigned int>("random_seed", seed, 0);

  pcl::PolygonMesh mesh;
  if(pcl::io::loadPLYFile(path, mesh) == -1)
  {
    ROS_ERROR("Failed to load PLY file");
    return -1;
  }

  srand(seed);

  colorized_mesh_display::ColorizedMeshStamped colorized_mesh;
  colorized_mesh.mesh = colorized_mesh_display::fromPCLPolygonMesh(mesh);
  colorized_mesh.header.frame_id = base_frame;
  colorized_mesh.header.stamp = ros::Time::now();

  ros::Publisher pub = nh.advertise<colorized_mesh_display::ColorizedMeshStamped>("colorized_mesh", 1, true);
  pub.publish(colorized_mesh);

  ros::spin();

  return 0;
}
