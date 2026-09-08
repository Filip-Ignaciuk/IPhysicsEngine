#include "particlelink.hpp"

IPhysics::real IPhysics::ParticleLink::CurrentLength() const{
    Vector3 relativePosition = particles[0]->GetPosition() - particles[1]->GetPosition();
    return relativePosition.Magnitude();
}

unsigned IPhysics::ParticleCable::AddContact(ParticleContact* contact, unsigned limit) const{
    real length = CurrentLength();

    if (length < maxLength){
        return 0;
    }

    contact->particles[0] = particles[0];
    contact->particles[1] = particles[1];

    Vector3 normal = particles[1]->GetPosition() - particles[0]->GetPosition();
    normal.Normalise();
    contact->contactNormal = normal;

    contact->penetration = length - maxLength;
    contact->restitution = restitution;

    return 1;
}

IPhysics::real IPhysics::ParticleRod::CurrentLength() const{
    Vector3 displacement =  particles[0]->GetPosition() - particles[1]->GetPosition();
    return displacement.Magnitude();
}

unsigned IPhysics::ParticleRod::AddContact(ParticleContact* contact, unsigned limit) const{
    real currentLength = CurrentLength();
    if (currentLength == length){
        return 0;
    }

    contact->particles[0] = particles[0];
    contact->particles[1] = particles[1];

    Vector3 normal = particles[1]->GetPosition() - particles[0]->GetPosition();
    normal.Normalise();
    if (currentLength > length){
        contact->contactNormal = normal;
        contact->penetration = currentLength - length;
    }
    else{
        contact->contactNormal = normal * -1;
        contact->penetration = length - currentLength;
    }

    contact->restitution = 0;

    return 1;
}