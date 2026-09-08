#ifndef IPHYSICS_DOWNWARDGRAVITY_HPP
#define IPHYSICS_DOWNWARDGRAVITY_HPP

#include "forcegenerator.hpp"

namespace IPhysics {
    class DownwardGravity : public ForceGenerator{
    public:
        // Constructors
        explicit DownwardGravity(const Vector3& _gravity);

        // Mutators
        void UpdateForce(RigidBody* _rigidBody, real _duration) override;
    private:
        Vector3 m_gravity;
    };
}

#endif
