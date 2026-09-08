#pragma once
#include "particle.hpp"

namespace IPhysics{
    class ParticleContactResolver;

    class ParticleContact{

        friend class ParticleContactResolver;

        public:
        Particle* particles[2];
        Vector3 particleMovement[2];

        real restitution;

        real penetration;

        Vector3 contactNormal;

        protected:
        void Resolve(real duration);

        real CalculateSeparatingVelocity() const;

        private:
        void ResolveVelocity(real duration);
        void ResolveInterpretation(real duration);
    };

    class ParticleContactResolver{
        protected:
        unsigned iterations;
        unsigned iterationsUsed;
        public:
        ParticleContactResolver(unsigned iterations);
        void SetIterations(unsigned iterations);
        void ResolveContacts(ParticleContact* contactArray, unsigned numberOfContacts, real duration);

    };

    class ParticleContactGenerator
    {
        public:
        virtual unsigned AddContact(ParticleContact* contact, unsigned limit) const = 0;
    };




}

