#pragma once
#include <vector>

#include "particle.hpp"

namespace IPhysicsEngine{
    class ParticleForceGenerator{
        public:
            virtual void UpdateForce(Particle* _particle, real _duration) = 0;
    };

    struct ParticleForceRegistration
    {
        Particle* particle;
        ParticleForceGenerator* particleForceGenerator;

        bool operator==(const ParticleForceRegistration& _other) const {
            return particle == _other.particle && particleForceGenerator == _other.particleForceGenerator;
        }
    };
    class ParticleForceRegistry{
        protected:
        

        std::vector<ParticleForceRegistration>* registrations;
        public:
        ParticleForceRegistry();
        void Add(Particle* _particle, ParticleForceGenerator* _particleForceGenerator);
        void Remove(Particle* _particle, ParticleForceGenerator* _particleForceGenerator);

        std::vector<ParticleForceRegistration>* GetRegistrations();

        void Clear();
        void UpdateForces(real _duration);
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
        const real gravitationalConstant = 0.000000000066743015;
        std::vector<ParticleForceRegistration>* registrations;
        public:
        ParticleRealGravity(std::vector<ParticleForceRegistration>* _registrations);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    


}