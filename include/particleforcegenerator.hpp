#pragma once
#include <vector>

#include "particle.hpp"

namespace IPhysicsEngine{
    class ParticleForceGenerator{
        public:
            virtual void UpdateForce(Particle* _particle, real _duration) = 0;
    };


    class ParticleGravity : public ParticleForceGenerator{
        private:
        Vector3 gravity;
        public:
        ParticleGravity(const Vector3& _gravity);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleDrag : public ParticleForceGenerator{
        private:
        real k1;
        real k2;
        public:
        ParticleDrag(const real& _k1, const real& _k2);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleRealGravity: public ParticleForceGenerator{
        private:
        real k1;
        real k2;
        public:
        ParticleDrag(const real& _k1, const real& _k2);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleForceRegistry{
        protected:
        struct ParticleForceRegistration
        {
            Particle* particle;
            ParticleForceGenerator* particleForceGenerator;

            bool operator==(const ParticleForceRegistration& _other) const {
                return particle == _other.particle && particleForceGenerator == _other.particleForceGenerator;
            }
        };

        std::vector<ParticleForceRegistration> registrations;
        public:
        void Add(Particle* _particle, ParticleForceGenerator* _particleForceGenerator);
        void Remove(Particle* _particle, ParticleForceGenerator* _particleForceGenerator);
        void Clear();
        void UpdateForces(real _duration);
    };


}