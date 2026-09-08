#include "cudagravity.cuh"
#include <cuda/cmath>
#include <cuda_runtime.h>
#include <iostream>

#include "precision.hpp"


__global__ void GravityCalculator2D(
    IPhysics::real* _positionsX,
    IPhysics::real* _positionsY,
    IPhysics::real* _masses,
    IPhysics::real* _forcesX,
    IPhysics::real* _forcesY,
    int _size,
    IPhysics::real _gravityConstant) {

    int workIndex = threadIdx.x + blockDim.x * blockIdx.x;

    int i = workIndex / _size;
    int j = workIndex % _size;
  
    
    if(workIndex < _size * _size && (i != j))
    {
        IPhysics::real totalMass = _masses[i] * _masses[j];

        IPhysics::real distanceX = _positionsX[j] - _positionsX[i];
        IPhysics::real distanceY = _positionsY[j] - _positionsY[i];

        IPhysics::real distanceMagnitude 
            = RealSqrt(distanceX * distanceX + distanceY * distanceY);

        const IPhysics::real forceMagnitude = -1 * _gravityConstant
            * totalMass / (distanceMagnitude * distanceMagnitude * distanceMagnitude);

        const IPhysics::real totalForceX = distanceX * forceMagnitude;
        const IPhysics::real totalForceY = distanceY * forceMagnitude;

        atomicAdd(&_forcesX[j], totalForceX);
        atomicAdd(&_forcesY[j], totalForceY);
    }
}

// Constructors
IPhysics::CudaGravity::CudaGravity(real _gravityConstant) :
    Gravity(_gravityConstant) {
}

// Deconstructors
IPhysics::CudaGravity::~CudaGravity(){
    cudaFree(m_positionsX);
    cudaFree(m_positionsY);
    cudaFree(m_masses);
    cudaFree(m_forcesX);
    cudaFree(m_forcesY);
}


// Mutators
void IPhysics::CudaGravity::AddObject(Object* _object){
    m_rigidBodies.emplace_back(_object->GetComponent<RigidBody>());
    UpdateParameters();
}

void IPhysics::CudaGravity::RemoveObject(Object* _object){
    std::erase(m_rigidBodies, _object->GetComponent<RigidBody>());
    UpdateParameters();
}

void IPhysics::CudaGravity::UpdateForce(RigidBody* _rigidBody, real _duration){
    EnsureCapacity(m_rigidBodiesSize);
    // Ensure there is more than one body.
    if(2 > m_rigidBodiesSize){
        return;
    }

    int threads = 256;

    for (int i = 0; i < m_rigidBodiesSize; i++) {       
        m_positionsX[i] = m_rigidBodies[i]->GetPosition().x;
        m_positionsY[i] = m_rigidBodies[i]->GetPosition().y;

        m_masses[i] = m_rigidBodies[i]->GetMass();

        m_forcesX[i] = 0;
        m_forcesY[i] = 0;
    }

    GravityCalculator2D<<<m_blocks, threads>>>(
        m_positionsX, 
        m_positionsY, 
        m_masses, 
        m_forcesX, 
        m_forcesY, 
        m_rigidBodiesSize,
        m_gravityConstant);

    cudaDeviceSynchronize();

    for (int i = 0; i < m_rigidBodiesSize; i++){
        m_rigidBodies[i]->AddForce({m_forcesX[i], m_forcesY[i], 0});
    }    
}

void IPhysics::CudaGravity::EnsureCapacity(long long _size){
    if(m_isAllocated){
        return;
    }

    cudaMallocManaged(&m_positionsX, _size * sizeof(real));
    cudaMallocManaged(&m_positionsY, _size * sizeof(real));
    cudaMallocManaged(&m_masses,     _size * sizeof(real));
    cudaMallocManaged(&m_forcesX,    _size * sizeof(real));
    cudaMallocManaged(&m_forcesY,    _size * sizeof(real));
}

void IPhysics::CudaGravity::UpdateParameters(){
    m_rigidBodiesSize = m_rigidBodies.size();
    m_totalThreads = m_rigidBodiesSize * m_rigidBodiesSize;
    m_blocks = cuda::ceil_div(m_totalThreads, m_rigidBodiesSize);
}
