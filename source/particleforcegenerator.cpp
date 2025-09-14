#include "particleforcegenerator.hpp"
#include <algorithm>

void IPhysicsEngine::ParticleForceRegistry::Add(Particle* _particle, ParticleForceGenerator* _particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = _particle;
    particleForceRegistration.particleForceGenerator = _particleForceGenerator;
    registrations.emplace_back(particleForceRegistration);
}

void IPhysicsEngine::ParticleForceRegistry::Remove(Particle* _particle, ParticleForceGenerator* _particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = _particle;
    particleForceRegistration.particleForceGenerator = _particleForceGenerator;
    registrations.erase(std::remove(registrations.begin(), registrations.end(), value_to_remove), registrations.end());
}