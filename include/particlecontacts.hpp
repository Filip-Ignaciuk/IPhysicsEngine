#pragma once
#include "particle.hpp"

namespace IPhysicsEngine{
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
        void Resolve(real _duration);

        real CalculateSeparatingVelocity() const;

        private:
        void ResolveVelocity(real _duration);
        void ResolveInterpretation(real _duration);
    };

    class ParticleContactResolver{
        protected:
        unsigned iterations;
        unsigned iterationsUsed;
        public:
        ParticleContactResolver(unsigned _iterations);
        void SetIterations(unsigned _iterations);
        void ResolveContacts(ParticleContact* _contactArray, unsigned _numberOfContacts, real _duration);

    };

    class ParticleContactGenerator
    {
        public:
        virtual unsigned AddContact(ParticleContact* _contact, unsigned _limit) const = 0;
    };




}

