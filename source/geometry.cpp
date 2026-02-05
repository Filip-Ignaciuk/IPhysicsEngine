#include "geometry.hpp"

IPhysicsEngine::Geometry::Geometry(){

}

Mesh IPhysicsEngine::Geometry::GetMesh(){
    return m_mesh;
}

IPhysicsEngine::real IPhysicsEngine::Geometry::GetScale(){
    return m_scale;
}

Color IPhysicsEngine::Geometry::GetColor(){
    return m_color;
}

void IPhysicsEngine::Geometry::SetMesh(Mesh _mesh){
    m_mesh = _mesh;
}

void IPhysicsEngine::Geometry::SetScale(float _scale){
    m_scale = _scale;
}

void IPhysicsEngine::Geometry::SetColor(Color _color){
    m_color = _color;
}