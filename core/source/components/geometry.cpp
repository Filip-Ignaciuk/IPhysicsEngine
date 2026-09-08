#include "components/geometry.hpp"

IPhysics::Geometry::Geometry() {}

Mesh* IPhysics::Geometry::GetMesh() { return m_mesh; }

IPhysics::real IPhysics::Geometry::GetScale() { return m_scale; }

Color IPhysics::Geometry::GetColor() { return m_color; }

void IPhysics::Geometry::SetMesh(Mesh* mesh) { m_mesh = mesh; }

void IPhysics::Geometry::SetScale(float scale) { m_scale = scale; }

void IPhysics::Geometry::SetColor(Color color) { m_color = color; }