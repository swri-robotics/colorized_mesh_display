#include <colorized_mesh_display/colorized_mesh_visual.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <boost/lexical_cast.hpp>

static uint32_t count = 0;

namespace colorized_mesh_display
{

ColorizedMeshVisual::ColorizedMeshVisual(Ogre::SceneManager* scene_manager,
                                         Ogre::SceneNode* parent_node)
  : scene_manager_(scene_manager)
  , material_name_("BaseWhiteNoLighting")
{
  if (!parent_node)
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  frame_node_ = parent_node->createChildSceneNode();
  manual_object_ = scene_manager_->createManualObject("ColorizedMesh" + boost::lexical_cast<std::string>(count++));
}

ColorizedMeshVisual::~ColorizedMeshVisual()
{
  scene_manager_->destroyManualObject(manual_object_);
  scene_manager_->destroySceneNode(frame_node_);
}

void ColorizedMeshVisual::visualizeMesh(const colorized_mesh_display::ColorizedMesh& msg)
{
  manual_object_->clear();

  // Begin adding mesh information
  manual_object_->estimateVertexCount(msg.vertices.size());
  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Add the vertices
  for(std::size_t i = 0; i < msg.vertices.size(); ++i)
  {
    const geometry_msgs::Point32& vertex = msg.vertices[i];
    const geometry_msgs::Vector3& normal = msg.vertex_normals[i];
    const std_msgs::ColorRGBA& color = msg.vertex_colors[i];

    manual_object_->position(vertex.x, vertex.y, vertex.z);
    manual_object_->normal(normal.x, normal.y, normal.z);
    manual_object_->colour(color.r, color.g, color.b, color.a);
  }

  // Add the triangles
  for(std::size_t i = 0; i < msg.triangles.size(); ++i)
  {
    const shape_msgs::MeshTriangle& tri = msg.triangles[i];
    manual_object_->triangle(tri.vertex_indices[0], tri.vertex_indices[1], tri.vertex_indices[2]);
  }

  // Stop adding mesh information
  manual_object_->end();

  // Attach the mesh object to the frame node
  frame_node_->attachObject(manual_object_);
}

void ColorizedMeshVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void ColorizedMeshVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

} // namepsace colorized_mesh_display
