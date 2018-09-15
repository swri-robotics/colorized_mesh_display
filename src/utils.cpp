#include <colorized_mesh_display/utils.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>

namespace colorized_mesh_display
{

colorized_mesh_display::ColorizedMesh fromPCLPolygonMesh(const pcl::PolygonMesh& mesh)
{
  // Check which fields exist
  bool has_color = false;
  bool has_normal_x = false;
  bool has_normal_y = false;
  bool has_normal_z = false;
  for(const pcl::PCLPointField& field : mesh.cloud.fields)
  {
    if(field.name == "rgb")
    {
      has_color = true;
    }
    else if(field.name == "normal_x")
    {
      has_normal_x = true;
    }
    else if(field.name == "normal_y")
    {
      has_normal_y = true;
    }
    else if(field.name == "normal_z")
    {
      has_normal_z = true;
    }
  }

  // Convert the message into a Point Cloud object
  pcl::PointCloud<pcl::PointXYZRGBNormal> vertex_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_cloud);

  // Create the output object
  colorized_mesh_display::ColorizedMesh out;
  out.vertices.reserve(vertex_cloud.size());
  out.vertex_normals.reserve(vertex_cloud.size());
  out.vertex_colors.reserve(vertex_cloud.size());
  out.triangles.reserve(mesh.polygons.size());

  // Add the vertex information
  for(std::size_t i = 0; i < vertex_cloud.points.size(); ++i)
  {
    const pcl::PointXYZRGBNormal& pt = vertex_cloud.points[i];

    // Add the vertex
    geometry_msgs::Point32 vertex;
    vertex.x = pt.x;
    vertex.y = pt.y;
    vertex.z = pt.z;
    out.vertices.push_back(vertex);

    // Add the normal
    geometry_msgs::Vector3 normal;
    if(has_normal_x && has_normal_y && has_normal_z)
    {
      normal.x = static_cast<double>(pt.normal_x);
      normal.y = static_cast<double>(pt.normal_y);
      normal.z = static_cast<double>(pt.normal_z);
    }
    else
    {
      normal.x = 0;
      normal.y = 0;
      normal.z = 1;
    }
    out.vertex_normals.push_back(normal);

    // Add the color
    std_msgs::ColorRGBA color;
    if(has_color)
    {
      color.r = pt.r;
      color.g = pt.g;
      color.b = pt.b;
      color.a = 1.0f;
    }
    else
    {
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      color.a = 1.0f;
    }
    out.vertex_colors.push_back(color);
  }

  // Add the triangle information
  for(std::size_t i = 0; i < mesh.polygons.size(); ++i)
  {
    const pcl::Vertices& poly = mesh.polygons[i];
    for(std::size_t j = 0; j < poly.vertices.size() - 2; ++j)
    {
      shape_msgs::MeshTriangle triangle;
      triangle.vertex_indices = {{poly.vertices[0], poly.vertices[j + 1], poly.vertices[j + 2]}};
      out.triangles.push_back(std::move(triangle));
    }
  }

  return out;
}

} // namespace colorized_mesh_display
