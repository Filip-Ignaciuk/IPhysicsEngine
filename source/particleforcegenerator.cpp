#include "particleforcegenerator.hpp"
#include <algorithm>

IPhysicsEngine::ParticleForceRegistry::ParticleForceRegistry(){
    registrations = new std::vector<ParticleForceRegistration>;
}


void IPhysicsEngine::ParticleForceRegistry::Add(Particle* _particle, ParticleForceGenerator* _particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = _particle;
    particleForceRegistration.particleForceGenerator = _particleForceGenerator;
    registrations->emplace_back(particleForceRegistration);
}

void IPhysicsEngine::ParticleForceRegistry::Remove(Particle* _particle, ParticleForceGenerator* _particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = _particle;
    particleForceRegistration.particleForceGenerator = _particleForceGenerator;
    registrations->erase(std::remove(registrations->begin(), registrations->end(), particleForceRegistration), registrations->end());
}

std::vector<IPhysicsEngine::ParticleForceRegistration>* IPhysicsEngine::ParticleForceRegistry::GetRegistrations(){    
    return registrations;
}

void IPhysicsEngine::ParticleForceRegistry::Clear(){
    registrations->clear();
}

void IPhysicsEngine::ParticleForceRegistry::UpdateForces(real _duration){

    for (unsigned i = 0; i < registrations->size(); i++)
    {
        (*registrations)[i].particleForceGenerator->UpdateForce((*registrations)[i].particle, _duration);
    }
    
}

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

IPhysicsEngine::ParticleRealGravity::ParticleRealGravity(std::vector<ParticleForceRegistration>* _registrations, real _gravitationalConstant){
    registrations = _registrations;
    gravitationalConstant = _gravitationalConstant;
}

void IPhysicsEngine::ParticleRealGravity::UpdateForce(Particle* _particle, real _duration){
    for (unsigned i = 0; i < registrations->size(); i++)
    {
        ParticleForceRegistration registeration = (*registrations)[i];
        if (registeration.particleForceGenerator == this && registeration.particle != _particle){
            Vector3 displacement = _particle->GetPosition() - registeration.particle->GetPosition();
            real denominator = displacement.Magnitude() * displacement.Magnitude() * displacement.Magnitude();
            Vector3 force = displacement * (-1 * gravitationalConstant * _particle->GetMass() * registeration.particle->GetMass() /  denominator);
            _particle->AddForce(force);
        }
    }
}

IPhysicsEngine::ParticleSpring::ParticleSpring(Particle* _otherParticle, real _springConstant, real _restLength){
    otherParticle = _otherParticle;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysicsEngine::ParticleSpring::UpdateForce(Particle* _particle, real _duration){
    Vector3 direction;
    Vector3 force;
    direction = _particle->GetPosition() - otherParticle->GetPosition();
    real magnitude = direction.Magnitude();
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    _particle->AddForce(force);

}

IPhysicsEngine::ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 _anchoredPosition, real _springConstant, real _restLength){
    anchoredPosition = _anchoredPosition;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysicsEngine::ParticleAnchoredSpring::UpdateForce(Particle* _particle, real _duration){
    Vector3 direction;
    Vector3 force;
    direction = _particle->GetPosition() - anchoredPosition;
    real magnitude = direction.Magnitude();
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    _particle->AddForce(force);

}

IPhysicsEngine::ParticleBungee::ParticleBungee(Particle* _otherParticle, real _springConstant, real _restLength){
    otherParticle = _otherParticle;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysicsEngine::ParticleBungee::UpdateForce(Particle* _particle, real _duration){
    Vector3 direction;
    Vector3 force;
    direction = _particle->GetPosition() - otherParticle->GetPosition();
    real magnitude = direction.Magnitude();
    if (magnitude <= restLength){
        return;
    }
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    _particle->AddForce(force);

}

IPhysicsEngine::ParticleAnchoredBungee::ParticleAnchoredBungee(Vector3 _anchoredPosition, real _springConstant, real _restLength){
    anchoredPosition = _anchoredPosition;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysicsEngine::ParticleAnchoredBungee::UpdateForce(Particle* _particle, real _duration){
    Vector3 direction;
    Vector3 force;
    direction = _particle->GetPosition() - anchoredPosition;
    real magnitude = direction.Magnitude();
    if (magnitude <= restLength){
        return;
    }
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    _particle->AddForce(force);

}