#include "particleforcegenerator.hpp"
#include <algorithm>

IPhysicsEngine::ParticleGravity::ParticleGravity(const Vector3& _gravity){
    gravity = _gravity;
}

void IPhysicsEngine::ParticleGravity::UpdateForce(Particle* _particle, real _duration){
    if (!_particle->GetInverseMass()){
        return;
    }
    _particle->AddForce(gravity * _particle->GetMass());
}

IPhysicsEngine::ParticleDrag::ParticleDrag(const real& _k1, const real& _k2){
    k1 = _k1;
    k2 = _k2;
}

void IPhysicsEngine::ParticleDrag::UpdateForce(Particle* _particle, real _duration){
    if (!_particle->GetInverseMass()){
        return;
    }
    Vector3 velocity = _particle->GetVelocity();
    velocity.Normalise();
    _particle->AddForce( velocity * -1 * (k1 * velocity.Magnitude() + k2 * velocity.Magnitude() * velocity.Magnitude()) );
}

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
    registrations.erase(std::remove(registrations.begin(), registrations.end(), particleForceRegistration), registrations.end());
}

void IPhysicsEngine::ParticleForceRegistry::Clear(){
    registrations.clear();
}

void IPhysicsEngine::ParticleForceRegistry::UpdateForces(real _duration){

    for (unsigned i = 0; i < registrations.size(); i++)
    {
        registrations[i].particleForceGenerator->UpdateForce(registrations[i].particle, _duration);
    }
    
}