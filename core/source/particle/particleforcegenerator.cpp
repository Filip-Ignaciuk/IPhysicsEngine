#include "particleforcegenerator.hpp"
#include <algorithm>

IPhysics::ParticleForceRegistry::ParticleForceRegistry(){
    std::vector<ParticleForceRegistration> temporary;
    registrations = temporary;
}


void IPhysics::ParticleForceRegistry::Add(Particle* _particle, ParticleForceGenerator* _particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = _particle;
    particleForceRegistration.particleForceGenerator = _particleForceGenerator;
    registrations.emplace_back(particleForceRegistration);
}

void IPhysics::ParticleForceRegistry::Remove(Particle* _particle, ParticleForceGenerator* _particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = _particle;
    particleForceRegistration.particleForceGenerator = _particleForceGenerator;
    registrations.erase(std::remove(registrations.begin(), registrations.end(), particleForceRegistration), registrations.end());
}

std::vector<IPhysics::ParticleForceRegistration>* IPhysics::ParticleForceRegistry::GetRegistrations(){    
    return &registrations;
}

void IPhysics::ParticleForceRegistry::Clear(){
    registrations.clear();
}

void IPhysics::ParticleForceRegistry::UpdateForces(real _duration){
    std::vector<ParticleForceRegistration>::iterator iterator = registrations.begin();
    while (iterator != registrations.end())
    {
        ParticleForceRegistration particleForceRegistery = *iterator;
        particleForceRegistery.particleForceGenerator->UpdateForce(particleForceRegistery.particle, _duration);
        ++iterator;
    }
    
    
}

IPhysics::ParticleGravity::ParticleGravity(const Vector3& _gravity){
    gravity = _gravity;
}

void IPhysics::ParticleGravity::UpdateForce(Particle* _particle, real _duration){
    if (!_particle->GetInverseMass()){
        return;
    }
    _particle->AddForce(gravity * _particle->GetMass());
}

IPhysics::ParticleDrag::ParticleDrag(const real& _k1, const real& _k2){
    k1 = _k1;
    k2 = _k2;
}

void IPhysics::ParticleDrag::UpdateForce(Particle* _particle, real _duration){
    if (!_particle->GetInverseMass()){
        return;
    }
    Vector3 velocity = _particle->GetVelocity();
    velocity.Normalise();
    _particle->AddForce( velocity * -1 * (k1 * velocity.Magnitude() + k2 * velocity.Magnitude() * velocity.Magnitude()) );
}

IPhysics::ParticleRealGravity::ParticleRealGravity(std::vector<ParticleForceRegistration>* _registrations, real _gravitationalConstant){
    registrations = _registrations;
    gravitationalConstant = _gravitationalConstant;
}

void IPhysics::ParticleRealGravity::UpdateForce(Particle* _particle, real _duration){
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

IPhysics::ParticleSpring::ParticleSpring(Particle* _otherParticle, real _springConstant, real _restLength){
    otherParticle = _otherParticle;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysics::ParticleSpring::UpdateForce(Particle* _particle, real _duration){
    Vector3 direction;
    Vector3 force;
    direction = _particle->GetPosition() - otherParticle->GetPosition();
    real magnitude = direction.Magnitude();
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    _particle->AddForce(force);

}

IPhysics::ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 _anchoredPosition, real _springConstant, real _restLength){
    anchoredPosition = _anchoredPosition;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysics::ParticleAnchoredSpring::UpdateForce(Particle* _particle, real _duration){
    Vector3 direction;
    Vector3 force;
    direction = _particle->GetPosition() - anchoredPosition;
    real magnitude = direction.Magnitude();
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    _particle->AddForce(force);

}

IPhysics::ParticleBungee::ParticleBungee(Particle* _otherParticle, real _springConstant, real _restLength){
    otherParticle = _otherParticle;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysics::ParticleBungee::UpdateForce(Particle* _particle, real _duration){
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

IPhysics::ParticleAnchoredBungee::ParticleAnchoredBungee(Vector3 _anchoredPosition, real _springConstant, real _restLength){
    anchoredPosition = _anchoredPosition;
    springConstant = _springConstant;
    restLength = _restLength;
}

void IPhysics::ParticleAnchoredBungee::UpdateForce(Particle* _particle, real _duration){
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

IPhysics::ParticleBuoyancy::ParticleBuoyancy(real _maxDepth, real _volume, real _waterHeight, real _liquidDensity){
    maxDepth = _maxDepth;
    volume = _volume;
    waterHeight = _waterHeight;
    liquidDensity = _liquidDensity;
}

void IPhysics::ParticleBuoyancy::UpdateForce(Particle* _particle, real _duration){
    real depth = _particle->GetPosition().GetY();
    if (depth <= waterHeight - maxDepth){
        _particle->AddForce(Vector3(0, volume * liquidDensity, 0));
    }
    else if (depth >= waterHeight + maxDepth){
        return;
    }
    else{
        _particle->AddForce(Vector3(0, ((depth - maxDepth - waterHeight) / 2 * maxDepth) * volume * liquidDensity, 0));
    }
}

IPhysics::ParticleFakeAnchoredSpring::ParticleFakeAnchoredSpring(Vector3 _anchoredPosition, real _springConstant, real _damping){
    anchoredPosition = _anchoredPosition;
    springConstant = _springConstant;
    damping = _damping;
}

void IPhysics::ParticleFakeAnchoredSpring::UpdateForce(Particle* _particle, real _duration){
    Vector3 position = _particle->GetPosition() - anchoredPosition;
    real gamma = 0.5f * RealSqrt(4 * springConstant - damping * damping);
    if (!gamma){
        return;
    }
    Vector3 c = position * (damping / ( 2.0f * gamma)) + _particle->GetVelocity() * (1.0f / gamma);

    Vector3 target = position * RealCos(gamma * _duration) + c * RealSin(gamma * _duration);
    target *= RealExp(-0.5f * _duration * damping);

    Vector3 acceleration = (target - position) * ( 1.0f / _duration * _duration ) - _particle->GetVelocity() * _duration;
    _particle->AddForce(acceleration * _particle->GetMass());
}