#include "cudabarneshutgravity.cuh"

#include <cuda_runtime.h>

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

__global__ void ComputeBoundingBox(){

}

__global__ void BuildHierarchicalDecomposition(){
  
}

__global__ void SumBodyInformation(){
  
}

__global__ void ApproxSortBodiesByDistance(){
  
}

__global__ void ComputeForces(){
  
}

__global__ void UpdateBodies(){
  
}
