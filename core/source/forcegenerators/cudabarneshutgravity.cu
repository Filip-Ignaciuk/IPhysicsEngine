#include "cudabarneshutgravity.cuh"
#include "gravity.hpp"

IPhysics::CUDABarnesHutGravity::CUDABarnesHutGravity(
    IPhysics::real gravityConstant,
    IPhysics::real thresholdValue) : Gravity(gravityConstant),
    m_thresholdValue(thresholdValue)
    {
}

void IPhysics::CUDABarnesHutGravity::AddObject(Object* _object) {
  m_rigidBodies.emplace_back(_object->GetComponent<RigidBody>());
}

void IPhysics::CUDABarnesHutGravity::RemoveObject(Object* _object) {
  std::erase(m_rigidBodies, _object->GetComponent<RigidBody>());
}

void IPhysics::CUDABarnesHutGravity::UpdateForce(RigidBody* _rigidBody, real _duration){

}