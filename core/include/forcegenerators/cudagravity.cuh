#ifndef IPHYSICS_CUDAGRAVITY_HPP
#define IPHYSICS_CUDAGRAVITY_HPP

#include "gravity.hpp"

namespace IPhysics {
    class CudaGravity : public Gravity {
    public:
        explicit CudaGravity(IPhysics::real _gravityConstant);

        ~CudaGravity();

        void AddObject(IPhysics::Object* _object) override;
        void RemoveObject(IPhysics::Object* _object) override;
        void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;

    private:
        long long m_rigidBodiesSize = 0;
        long long m_totalThreads = 0;
        long long m_blocks = 0;

        real* m_positionsX = nullptr;
        real* m_positionsY = nullptr;
        real* m_masses = nullptr;
        real* m_forcesX = nullptr;
        real* m_forcesY = nullptr;
        bool m_isAllocated = false;

        void EnsureCapacity(long long _size);

        void UpdateParameters();
    };
}

#endif