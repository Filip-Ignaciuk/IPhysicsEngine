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

    class ParticleRealGravity : public ParticleForceGenerator{
        private:
        real gravitationalConstant;
        std::vector<ParticleForceRegistration>* registrations;
        public:
        ParticleRealGravity(std::vector<ParticleForceRegistration>* _registrations, real _gravitationalConstant);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleSpring : public ParticleForceGenerator{
        private:
        Particle* otherParticle;
        real springConstant;
        real restLength;
        public:
        ParticleSpring(Particle* _otherParticle, real _springConsant, real _restLength);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleAnchoredSpring : public ParticleForceGenerator{
        private:
        Vector3 anchoredPosition;
        real springConstant;
        real restLength;
        public:
        ParticleAnchoredSpring(Vector3 _anchoredPosition, real _springConsant, real _restLength);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleBungee : public ParticleForceGenerator{
        private:
        Particle* otherParticle;
        real springConstant;
        real restLength;
        public:
        ParticleBungee(Particle* _otherParticle, real _springConsant, real _restLength);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleAnchoredBungee : public ParticleForceGenerator{
        private:
        Vector3 anchoredPosition;
        real springConstant;
        real restLength;
        public:
        ParticleAnchoredBungee(Vector3 _anchoredPosition, real _springConsant, real _restLength);
        void UpdateForce(Particle* _particle, real _duration) override;
    };

    class ParticleBuoyancy : public ParticleForceGenerator{
        private:
        real maxDepth;
        real volume;
        real waterHeight;
        real liquidDensity;
        public:
        ParticleBuoyancy(real _maxDepth, real _volume, real _waterHeight, real _liquidDensity);
        void UpdateForce(Particle* _particle, real _duration) override;
    };
    


}