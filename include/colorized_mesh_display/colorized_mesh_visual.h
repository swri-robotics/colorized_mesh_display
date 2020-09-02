#ifndef COLORIZED_MESH_VISUAL_H
#define COLORIZED_MESH_VISUAL_H

//#include <colorized_mesh_display/ColorizedMesh.h>
#include <pcl/PolygonMesh.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class SceneNode;
class SceneManager;
class ManualObject;
class Entity;
}

namespace rviz
{
class MeshShape;
}

namespace colorized_mesh_display
{

class ColorizedMeshShape;

class ColorizedMeshVisual
{
public:
  ColorizedMeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node = nullptr);

  virtual ~ColorizedMeshVisual();

  void visualizeMesh(const pcl::PolygonMesh& msg);

  void setFramePosition(const Ogre::Vector3& position);

  void setFrameOrientation(const Ogre::Quaternion& orientation);

private:

  Ogre::SceneManager* scene_manager_;

  Ogre::SceneNode* frame_node_;

  Ogre::ManualObject* manual_object_;

  Ogre::Entity* entity_;

  std::string material_name_;
};

} // namespace colorized_mesh_display

#endif // COLORIZED_MESH_VISUAL_H
