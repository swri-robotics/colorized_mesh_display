#include <colorized_mesh_display/ColorizedMeshStamped.h>
#include <pcl/io/ply_io.h>
#include <ros/ros.h>
#include <random>

float generateRandomFloat()
{
  return float(rand()) / float(INT_MAX);
}

colorized_mesh_display::ColorizedMeshStamped toColorizedMesh(const pcl::PolygonMesh& in,
                                                             const std::string& base_frame)
{
  colorized_mesh_display::ColorizedMeshStamped out;
  out.header.frame_id = base_frame;
  out.header.stamp = ros::Time::now();

  pcl::PointCloud<pcl::PointNormal> vertex_cloud;
  pcl::fromPCLPointCloud2(in.cloud, vertex_cloud);

  // Add the vertex information
  for(std::size_t i = 0; i < vertex_cloud.points.size(); ++i)
  {
    const pcl::PointNormal& pt = vertex_cloud.points[i];
    // Add the vertex
    geometry_msgs::Point32 vertex;
    vertex.x = pt.x;
    vertex.y = pt.y;
    vertex.z = pt.z;
    out.mesh.vertices.push_back(vertex);

    // Add the normal
    geometry_msgs::Vector3 normal;
    normal.x = 0; //pt.normal_x;
    normal.y = 0; //pt.normal_y;
    normal.z = 1; //pt.normal_z;
    out.mesh.vertex_normals.push_back(normal);

    // Add the color
    std_msgs::ColorRGBA color;
    color.r = generateRandomFloat();
    color.g = generateRandomFloat();
    color.b = generateRandomFloat();
    color.a = 1.0f;
    out.mesh.vertex_colors.push_back(color);
  }

  // Add the triangle information
  for(std::size_t i = 0; i < in.polygons.size(); ++i)
  {
    const pcl::Vertices& poly = in.polygons[i];
    for(std::size_t j = 0; j < poly.vertices.size() - 2; ++j)
    {
      shape_msgs::MeshTriangle triangle;
      triangle.vertex_indices = {{poly.vertices[0], poly.vertices[j + 1], poly.vertices[j + 2]}};
      out.mesh.triangles.push_back(std::move(triangle));
    }
  }

  return out;
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

  colorized_mesh_display::ColorizedMeshStamped colorized_mesh = toColorizedMesh(mesh, base_frame);

  ros::Publisher pub = nh.advertise<colorized_mesh_display::ColorizedMeshStamped>("colorized_mesh", 1, true);
  pub.publish(colorized_mesh);

  ros::spin();

  return 0;
}
