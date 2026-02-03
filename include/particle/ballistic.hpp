#pragma once
#include "particle.hpp"

namespace IPhysicsEngine{
    class BallisticParticle : public Particle{
    public:
        BallisticParticle();
        BallisticParticle(Vector3 _position, real _damping, real _inverseMass);
        bool Integrate(real _duration) override;

    };
}