#ifndef IPHYSICS_GEOMETRY_HPP
#define IPHYSICS_GEOMETRY_HPP
#include "components/component.hpp"
#include "core.hpp"
#include "raylib.h"

namespace IPhysics {
class Geometry : public Component {
 private:
  Mesh* m_mesh;
  float m_scale;
  Color m_color;

 public:
  Geometry();

  Mesh* GetMesh();
  real GetScale();
  Color GetColor();

  void SetMesh(Mesh* mesh);
  void SetScale(float scale);
  void SetColor(Color color);
};
}  // namespace IPhysics

#endif