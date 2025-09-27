#include "particleworld.hpp"

IPhysicsEngine::ParticleWorld::ParticleWorld(unsigned _maxContacts, unsigned _iterations) : particleContactResolvers(_maxContacts), maxContacts(_iterations){
    contacts = new ParticleContact[maxContacts];
    calculateIterations = (_iterations == 0);
}

void IPhysicsEngine::ParticleWorld::StartFrame(){
    for (int i = 0; i < particles.size(); i++)
    {
        particles[i]->ClearAccumulator();
    }
}

unsigned IPhysicsEngine::ParticleWorld::GenerateContacts(){
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

void IPhysicsEngine::ParticleWorld::Integrate(real _duration){
    for (int i = 0; i < particles.size(); i++)
    {
        particles[i]->Integrate(_duration);
    }
    
}

void IPhysicsEngine::ParticleWorld::RunPhysics(real _duration){
    particleForceRegistry.UpdateForces(_duration);
    Integrate(_duration);

    unsigned usedContacts = GenerateContacts();

    if (usedContacts){
        if (calculateIterations){
            particleContactResolvers.SetIterations(usedContacts * 2);
            particleContactResolvers.ResolveContacts(contacts, usedContacts, _duration);
        }
    }
    
}

IPhysicsEngine::ParticleWorld::Particles& IPhysicsEngine::ParticleWorld::GetParticles(){
    return particles;
}

IPhysicsEngine::ParticleWorld::ContactGenerators& IPhysicsEngine::ParticleWorld::GetParticleContactGenerator(){
    return contactGenerators;
}

IPhysicsEngine::ParticleForceRegistry& IPhysicsEngine::ParticleWorld::GetParticleForceRegistry(){
    return particleForceRegistry;
}

void IPhysicsEngine::ParticleGroundContactGenerator::Init(IPhysicsEngine::ParticleWorld::Particles* _particles){
    particles = _particles;
}

unsigned IPhysicsEngine::ParticleGroundContactGenerator::AddContact(ParticleContact* _contact, unsigned _limit) const{
    unsigned count = 0;
    for (Particle* particle : *particles)
    {
        real yCoordinate = particle->GetPosition().GetY();
        if (yCoordinate < 0){
            _contact->contactNormal = Up;
            _contact->particles[0] = particle;
            _contact->particles[1] = nullptr;
            _contact->penetration = -yCoordinate;
            _contact->restitution = 0.2f;
            _contact++;
            count++;
        }
        if (count >= _limit){
        return count;
        }
    }
    return count;
}