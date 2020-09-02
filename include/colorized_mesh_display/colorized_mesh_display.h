#ifndef COLORIZED_MESH_DISPLAY_H
#define COLORIZED_MESH_DISPLAY_H

#include <rviz/message_filter_display.h>
#include <pcl_msgs/PolygonMesh.h>

namespace Ogre
{
class SceneNode;
}

namespace colorized_mesh_display
{

class ColorizedMeshVisual;

class ColorizedMeshDisplay : public rviz::MessageFilterDisplay<pcl_msgs::PolygonMesh>
{
  Q_OBJECT

public:

  ColorizedMeshDisplay();

  virtual ~ColorizedMeshDisplay();

protected:

  virtual void onInitialize() override;

  virtual void reset() override;

private:

  void processMessage(const pcl_msgs::PolygonMeshConstPtr& msg) override;

  std::shared_ptr<ColorizedMeshVisual> visual_;

};

} // namespace colorized_mesh_display

#endif // COLORIZED_MESH_DISPLAY_H
