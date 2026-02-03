#include "particlelink.hpp"

IPhysicsEngine::real IPhysicsEngine::ParticleLink::CurrentLength() const{
    Vector3 relativePosition = particles[0]->GetPosition() - particles[1]->GetPosition();
    return relativePosition.Magnitude();
}

unsigned IPhysicsEngine::ParticleCable::AddContact(ParticleContact* _contact, unsigned _limit) const{
    real length = CurrentLength();

    if (length < maxLength){
        return 0;
    }

    _contact->particles[0] = particles[0];
    _contact->particles[1] = particles[1];

    Vector3 normal = particles[1]->GetPosition() - particles[0]->GetPosition();
    normal.Normalise();
    _contact->contactNormal = normal;

    _contact->penetration = length - maxLength;
    _contact->restitution = restitution;

    return 1;
}

IPhysicsEngine::real IPhysicsEngine::ParticleRod::CurrentLength() const{
    Vector3 displacement =  particles[0]->GetPosition() - particles[1]->GetPosition();
    return displacement.Magnitude();
}

unsigned IPhysicsEngine::ParticleRod::AddContact(ParticleContact* _contact, unsigned _limit) const{
    real currentLength = CurrentLength();
    if (currentLength == length){
        return 0;
    }

    _contact->particles[0] = particles[0];
    _contact->particles[1] = particles[1];

    Vector3 normal = particles[1]->GetPosition() - particles[0]->GetPosition();
    normal.Normalise();
    if (currentLength > length){
        _contact->contactNormal = normal;
        _contact->penetration = currentLength - length;
    }
    else{
        _contact->contactNormal = normal * -1;
        _contact->penetration = length - currentLength;
    }

    _contact->restitution = 0;

    return 1;
}