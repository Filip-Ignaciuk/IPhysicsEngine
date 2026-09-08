#pragma once
#include <vector>

#include "particle.hpp"

namespace IPhysics{
    class ParticleForceGenerator{
        public:
        // Deconstructors
        virtual ~ParticleForceGenerator() = default;

        virtual void UpdateForce(Particle* particle, real duration) = 0;
    };

    struct ParticleForceRegistration
    {
        Particle* particle;
        ParticleForceGenerator* particleForceGenerator;

        bool operator==(const ParticleForceRegistration& other) const {
            return particle == other.particle && particleForceGenerator == other.particleForceGenerator;
        }
    };

    class ParticleForceRegistry{
        private:
        std::vector<ParticleForceRegistration> registrations;

        public:
        ParticleForceRegistry();
        void Add(Particle* particle, ParticleForceGenerator* particleForceGenerator);
        void Remove(Particle* particle, ParticleForceGenerator* particleForceGenerator);
        void Clear();
        void UpdateForces(real duration);
    
        std::vector<ParticleForceRegistration> GetRegistrations();
    };

    class ParticleGravity : public ParticleForceGenerator{
        private:
        Vector3 gravity;
        public:
        ParticleGravity(const Vector3& gravity);
        void UpdateForce(Particle* particle, real duration) override;
    };

    class ParticleDrag : public ParticleForceGenerator{
        private:
        real k1;
        real k2;
        public:
        ParticleDrag(const real& k1, const real& k2);
        void UpdateForce(Particle* particle, real duration) override;
    };

    class ParticleRealGravity : public ParticleForceGenerator{
        private:
        real gravitationalConstant;
        std::vector<ParticleForceRegistration>* registrations;
        public:
        ParticleRealGravity(std::vector<ParticleForceRegistration>* registrations, real gravitationalConstant);
        void UpdateForce(Particle* particle, real duration) override;
    };

    class ParticleSpring : public ParticleForceGenerator{
        private:
        Particle* otherParticle;
        real springConstant;
        real restLength;
        public:
        ParticleSpring(Particle* otherParticle, real springConsant, real restLength);
        void UpdateForce(Particle* particle, real duration) override;
    };

    class ParticleAnchoredSpring : public ParticleForceGenerator{
        private:
        Vector3 anchoredPosition;
        real springConstant;
        real restLength;
        public:
        ParticleAnchoredSpring(Vector3 anchoredPosition, real springConsant, real restLength);
        void UpdateForce(Particle* particle, real duration) override;
    };

    class ParticleBungee : public ParticleForceGenerator{
        private:
        Particle* otherParticle;
        real springConstant;
        real restLength;
        public:
        ParticleBungee(Particle* otherParticle, real springConsant, real restLength);
        void UpdateForce(Particle* particle, real duration) override;
    };

    class ParticleAnchoredBungee : public ParticleForceGenerator{
        private:
        Vector3 anchoredPosition;
        real springConstant;
        real restLength;
        public:
        ParticleAnchoredBungee(Vector3 anchoredPosition, real springConsant, real restLength);
        void UpdateForce(Particle* particle, real duration) override;
    };

    class ParticleBuoyancy : public ParticleForceGenerator{
        private:
        real maxDepth;
        real volume;
        real waterHeight;
        real liquidDensity;
        public:
        ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity);
        void UpdateForce(Particle* particle, real duration) override;
    };
    
    class ParticleFakeAnchoredSpring : public ParticleForceGenerator{
        private:
        Vector3 anchoredPosition;
        real springConstant;
        real damping;
        public:
        ParticleFakeAnchoredSpring(Vector3 anchoredPosition, real springConstant, real damping);
        void UpdateForce(Particle* particle, real duration) override;
    };

}