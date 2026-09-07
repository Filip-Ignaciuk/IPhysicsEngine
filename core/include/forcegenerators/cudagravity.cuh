#ifndef IPHYSICS_CUDAGRAVITY_HPP
#define IPHYSICS_CUDAGRAVITY_HPP

#include "gravity.hpp"

namespace IPhysics {
    class CudaGravity : public Gravity {
    public:
        explicit CudaGravity(IPhysics::real _gravityConstant);
        void AddObject(IPhysics::Object* _object) override;
        void RemoveObject(IPhysics::Object* _object) override;
        void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;
    private:
        __global__ void vecAdd(float* A, float* B, float* C);
    };
}

#endif