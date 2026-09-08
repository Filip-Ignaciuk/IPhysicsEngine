#ifndef IPHYSICS_GRAVITY_HPP
#define IPHYSICS_GRAVITY_HPP

#include "forcegenerator.hpp"

namespace IPhysics {
    class Gravity : public ForceGenerator{
    public:
        // Constructors
        explicit Gravity(const real& gravityConstant);

        // Mutators
        virtual void AddObject(Object* object);
        virtual void RemoveObject(Object* object);
        void UpdateForce(RigidBody* rigidBody, real duration) override;
    protected:
        real m_gravityConstant;
        std::vector<RigidBody*> m_rigidBodies;
    };
}

#endif
