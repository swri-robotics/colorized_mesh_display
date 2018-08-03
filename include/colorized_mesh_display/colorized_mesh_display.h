#ifndef COLORIZED_MESH_DISPLAY_H
#define COLORIZED_MESH_DISPLAY_H

#include <colorized_mesh_display/ColorizedMeshStamped.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace colorized_mesh_display
{

class ColorizedMeshVisual;

class ColorizedMeshDisplay : public rviz::MessageFilterDisplay<ColorizedMeshStamped>
{
  Q_OBJECT

public:

  ColorizedMeshDisplay();

  virtual ~ColorizedMeshDisplay();

protected:

  virtual void onInitialize() override;

  virtual void reset() override;

private:

  void processMessage(const ColorizedMeshStampedConstPtr& msg) override;

  std::shared_ptr<ColorizedMeshVisual> visual_;

};

} // namespace colorized_mesh_display

#endif // COLORIZED_MESH_DISPLAY_H
