#include "particleworld.hpp"

IPhysics::ParticleWorld::ParticleWorld(unsigned maxContacts,
                                       unsigned iterations)
    : particleContactResolvers(maxContacts), maxContacts(iterations) {
  contacts = new ParticleContact[maxContacts];
  calculateIterations = (iterations == 0);
}

void IPhysics::ParticleWorld::StartFrame() {
  for (int i = 0; i < particles.size(); i++) {
    particles[i]->ClearAccumulator();
  }
}

unsigned IPhysics::ParticleWorld::GenerateContacts() {
  unsigned limit = maxContacts;
  ParticleContact* nextContact = contacts;

  for (int i = 0; i < contactGenerators.size(); i++) {
    unsigned used = contactGenerators[i]->AddContact(nextContact, limit);
    limit -= used;
    nextContact += used;

    if (limit <= 0) {
      break;
    }
  }

  return maxContacts - limit;
}

void IPhysics::ParticleWorld::Integrate(real duration) {
  for (int i = 0; i < particles.size(); i++) {
    particles[i]->Integrate(duration);
  }
}

void IPhysics::ParticleWorld::RunPhysics(real duration) {
  particleForceRegistry.UpdateForces(duration);
  Integrate(duration);

  unsigned usedContacts = GenerateContacts();

  if (usedContacts) {
    if (calculateIterations) {
      particleContactResolvers.SetIterations(usedContacts * 2);
    }
    particleContactResolvers.ResolveContacts(contacts, usedContacts, duration);
  }
}

IPhysics::ParticleWorld::Particles& IPhysics::ParticleWorld::GetParticles() {
  return particles;
}

IPhysics::ParticleWorld::ContactGenerators&
IPhysics::ParticleWorld::GetParticleContactGenerator() {
  return contactGenerators;
}

IPhysics::ParticleForceRegistry&
IPhysics::ParticleWorld::GetParticleForceRegistry() {
  return particleForceRegistry;
}

void IPhysics::ParticleGroundContactGenerator::Init(
    IPhysics::ParticleWorld::Particles* particles, real restitution) {
  particles = particles;
  restitution = restitution;
}

unsigned IPhysics::ParticleGroundContactGenerator::AddContact(
    ParticleContact* contact, unsigned limit) const {
  unsigned count = 0;
  for (auto particle : *particles) {
    real yCoordinate = particle->GetPosition().GetY();
    if (yCoordinate < 0) {
      contact->contactNormal = Up;
      contact->particles[0] = particle;
      contact->particles[1] = nullptr;
      contact->penetration = -yCoordinate;
      contact->restitution = restitution;
      contact++;
      count++;
    }
    if (count >= limit) {
      return count;
    }
  }
  return count;
}