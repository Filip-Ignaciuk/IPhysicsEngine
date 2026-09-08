#ifndef IPHYSICS_GRAVITY_HPP
#define IPHYSICS_GRAVITY_HPP

#include "forcegenerator.hpp"

namespace IPhysics {
    class Gravity : public ForceGenerator{
    public:
        // Constructors
        explicit Gravity(const real& _gravityConstant);

        // Mutators
        virtual void AddObject(Object* _object);
        virtual void RemoveObject(Object* _object);
        void UpdateForce(RigidBody* _rigidBody, real _duration) override;
    protected:
        real m_gravityConstant;
        std::vector<RigidBody*> m_rigidBodies;
    };
}

#endif
