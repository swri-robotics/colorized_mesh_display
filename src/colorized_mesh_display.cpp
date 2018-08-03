#include <colorized_mesh_display/colorized_mesh_display.h>
#include <colorized_mesh_display/colorized_mesh_visual.h>
#include <ros/console.h>

namespace colorized_mesh_display
{

ColorizedMeshDisplay::ColorizedMeshDisplay()
{

}

ColorizedMeshDisplay::~ColorizedMeshDisplay()
{

}

void ColorizedMeshDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void ColorizedMeshDisplay::reset()
{
  MFDClass::reset();
  visual_.reset();
}

void ColorizedMeshDisplay::processMessage(const ColorizedMeshStampedConstPtr& msg)
{
  // Check the size of the incoming vertex buffers
  if(msg->mesh.vertices.size() != msg->mesh.vertex_normals.size() ||
     msg->mesh.vertices.size() != msg->mesh.vertex_colors.size())
  {
    ROS_ERROR("Vertex size does not match vertex normal or vertex color size");
    return;
  }

  // Get the transform
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                 msg->header.stamp,
                                                 position, orientation))
  {
    ROS_DEBUG("Error transforming into frame '%s'", msg->header.frame_id.c_str());
    return;
  }

  visual_.reset(new ColorizedMeshVisual(scene_manager_));
  visual_->visualizeMesh(msg->mesh);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

} // namespace colorized_mesh_display

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(colorized_mesh_display::ColorizedMeshDisplay, rviz::Display)
