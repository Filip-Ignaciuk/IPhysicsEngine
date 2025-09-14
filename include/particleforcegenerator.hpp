#pragma once
#include <vector>

#include "particle.hpp"

namespace IPhysicsEngine{
    class ParticleForceGenerator{
        public:
            virtual void UpdateForce(Particle* _particle, real _duration) = 0;
    };

    class ParticleForceRegistry{
        protected:
        struct ParticleForceRegistration
        {
            Particle* particle;
            ParticleForceGenerator* particleForceGenerator;
        };

        std::vector<ParticleForceRegistration> registrations;
        public:
        void Add(Particle* _particle, ParticleForceGenerator* _particleForceGenerator);
        void Remove(Particle* _particle, ParticleForceGenerator* _particleForceGenerator);
        void Clear();
        void UpdateForces(real _duration);
    };
}