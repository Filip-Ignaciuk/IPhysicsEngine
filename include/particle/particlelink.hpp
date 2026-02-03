#pragma once
#include "particlecontacts.hpp"

namespace IPhysicsEngine{
    class ParticleLink : public ParticleContactGenerator{
        public:
        Particle* particles[2];
        protected:
        real CurrentLength() const;
        virtual unsigned AddContact(ParticleContact* _contact, unsigned _limit) = 0;
    };

    class ParticleCable : public ParticleLink{
        public:
        real maxLength;
        real restitution;
        virtual unsigned AddContact(ParticleContact* _contact, unsigned _limit) const;
    };

    class ParticleRod : public ParticleLink{
        public:
        real length;
        real CurrentLength() const;
        virtual unsigned AddContact(ParticleContact* _contact, unsigned _limit) const;
    };
}