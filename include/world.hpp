#pragma once
#include <vector>

#include "rigidbody.hpp"
#include "forcegenerator.hpp"

namespace IPhysicsEngine
{
    class World{
        public:
        typedef std::vector<RigidBody*> Rigidbodies;

        void StartFrame();
        void RunPhysics(real _duration);

        ForceRegistry& GetParticleForceRegistry();

        Rigidbodies& GetRigidBodies();

        protected:
        Rigidbodies m_rigidBodies;
        ForceRegistry m_registery;
    };
}