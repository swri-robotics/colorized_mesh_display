#ifndef COLORIZED_MESH_DISPLAY_UTILS_H
#define COLORIZED_MESH_DISPLAY_UTILS_H

#include <colorized_mesh_display/ColorizedMesh.h>
#include <pcl/PolygonMesh.h>

namespace colorized_mesh_display
{

colorized_mesh_display::ColorizedMesh fromPCLPolygonMesh(const pcl::PolygonMesh& mesh);

} // namespace colorized_mesh_display

#endif // COLORIZED_MESH_DISPLAY_UTILS_H
