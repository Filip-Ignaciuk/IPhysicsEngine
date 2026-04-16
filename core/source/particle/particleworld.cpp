#include "particleworld.hpp"

IPhysics::ParticleWorld::ParticleWorld(unsigned _maxContacts, unsigned _iterations) : particleContactResolvers(_maxContacts), maxContacts(_iterations){
    contacts = new ParticleContact[maxContacts];
    calculateIterations = (_iterations == 0);
}

void IPhysics::ParticleWorld::StartFrame(){
    for (int i = 0; i < particles.size(); i++)
    {
        particles[i]->ClearAccumulator();
    }
}

unsigned IPhysics::ParticleWorld::GenerateContacts(){
    unsigned limit = maxContacts;
     ParticleContact* nextContact = contacts;

    for (int i = 0; i < contactGenerators.size(); i++)
    {
        unsigned used = contactGenerators[i]->AddContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        if (limit <= 0){
            break;
        }
    }
    
    return maxContacts - limit;
}

void IPhysics::ParticleWorld::Integrate(real _duration){
    for (int i = 0; i < particles.size(); i++)
    {
        particles[i]->Integrate(_duration);
    }
    
}

void IPhysics::ParticleWorld::RunPhysics(real _duration){
    particleForceRegistry.UpdateForces(_duration);
    Integrate(_duration);

    unsigned usedContacts = GenerateContacts();

    if (usedContacts){
        if (calculateIterations){
            particleContactResolvers.SetIterations(usedContacts * 2);
        }
        particleContactResolvers.ResolveContacts(contacts, usedContacts, _duration);
    }
    
}

IPhysics::ParticleWorld::Particles& IPhysics::ParticleWorld::GetParticles(){
    return particles;
}

IPhysics::ParticleWorld::ContactGenerators& IPhysics::ParticleWorld::GetParticleContactGenerator(){
    return contactGenerators;
}

IPhysics::ParticleForceRegistry& IPhysics::ParticleWorld::GetParticleForceRegistry(){
    return particleForceRegistry;
}

void IPhysics::ParticleGroundContactGenerator::Init(IPhysics::ParticleWorld::Particles* _particles, real _restitution){
    particles = _particles;
    restitution = _restitution;
}

unsigned IPhysics::ParticleGroundContactGenerator::AddContact(ParticleContact* _contact, unsigned _limit) const{
    unsigned count = 0;
    for (auto particle : *particles)
    {
        real yCoordinate = particle->GetPosition().GetY();
        if (yCoordinate < 0){
            _contact->contactNormal = Up;
            _contact->particles[0] = particle;
            _contact->particles[1] = nullptr;
            _contact->penetration = -yCoordinate;
            _contact->restitution = restitution;
            _contact++;
            count++;
        }
        if (count >= _limit){
        return count;
        }
    }
    return count;
}