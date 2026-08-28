#ifndef IPHYSICS_CUDABARNESHUTGRAVITY_HPP
#define IPHYSICS_CUDABARNESHUTGRAVITY_HPP

#include "gravity.hpp"

namespace IPhysics {
    class RealGravityBarnesHutCuda : public IPhysics::ForceGenerator {
    public:
        explicit RealGravityBarnesHutCuda(const IPhysics::real& _gravityConstant);

        void AddObject(IPhysics::Object* _object);
        void RemoveObject(IPhysics::Object* _object);

        void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;
    private:
        void CreateTree();
    };
}

#endif
