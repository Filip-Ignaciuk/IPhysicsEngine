#include "components/geometry.hpp"

IPhysics::Geometry::Geometry(){

}

Mesh* IPhysics::Geometry::GetMesh(){
    return m_mesh;
}

IPhysics::real IPhysics::Geometry::GetScale(){
    return m_scale;
}

Color IPhysics::Geometry::GetColor(){
    return m_color;
}

void IPhysics::Geometry::SetMesh(Mesh* _mesh){
    m_mesh = _mesh;
}

void IPhysics::Geometry::SetScale(float _scale){
    m_scale = _scale;
}

void IPhysics::Geometry::SetColor(Color _color){
    m_color = _color;
}