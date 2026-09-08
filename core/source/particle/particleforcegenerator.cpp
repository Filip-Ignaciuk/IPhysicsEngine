#include "particleforcegenerator.hpp"
#include <algorithm>

IPhysics::ParticleForceRegistry::ParticleForceRegistry(){
    std::vector<ParticleForceRegistration> temporary;
    registrations = temporary;
}


void IPhysics::ParticleForceRegistry::Add(Particle* particle, ParticleForceGenerator* particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = particle;
    particleForceRegistration.particleForceGenerator = particleForceGenerator;
    registrations.emplace_back(particleForceRegistration);
}

void IPhysics::ParticleForceRegistry::Remove(Particle* particle, ParticleForceGenerator* particleForceGenerator){
    ParticleForceRegistration particleForceRegistration{};
    particleForceRegistration.particle = particle;
    particleForceRegistration.particleForceGenerator = particleForceGenerator;
    registrations.erase(std::remove(registrations.begin(), registrations.end(), particleForceRegistration), registrations.end());
}

std::vector<IPhysics::ParticleForceRegistration> IPhysics::ParticleForceRegistry::GetRegistrations(){
    return &registrations;
}

void IPhysics::ParticleForceRegistry::Clear(){
    registrations.clear();
}

void IPhysics::ParticleForceRegistry::UpdateForces(real duration){
    std::vector<ParticleForceRegistration>::iterator iterator = registrations.begin();
    while (iterator != registrations.end())
    {
        ParticleForceRegistration particleForceRegistery = *iterator;
        particleForceRegistery.particleForceGenerator->UpdateForce(particleForceRegistery.particle, duration);
        ++iterator;
    }
    
    
}

IPhysics::ParticleGravity::ParticleGravity(const Vector3& gravity){
    gravity = gravity;
}

void IPhysics::ParticleGravity::UpdateForce(Particle* particle, real duration){
    if (!particle->GetInverseMass()){
        return;
    }
    particle->AddForce(gravity * particle->GetMass());
}

IPhysics::ParticleDrag::ParticleDrag(const real& k1, const real& k2){
    k1 = k1;
    k2 = k2;
}

void IPhysics::ParticleDrag::UpdateForce(Particle* particle, real duration){
    if (!particle->GetInverseMass()){
        return;
    }
    Vector3 velocity = particle->GetVelocity();
    velocity.Normalise();
    particle->AddForce( velocity * -1 * (k1 * velocity.Magnitude() + k2 * velocity.Magnitude() * velocity.Magnitude()) );
}

IPhysics::ParticleRealGravity::ParticleRealGravity(std::vector<ParticleForceRegistration>* registrations, real gravitationalConstant){
    registrations = registrations;
    gravitationalConstant = gravitationalConstant;
}

void IPhysics::ParticleRealGravity::UpdateForce(Particle* particle, real duration){
    for (unsigned i = 0; i < registrations->size(); i++)
    {
        ParticleForceRegistration registeration = (*registrations)[i];
        if (registeration.particleForceGenerator == this && registeration.particle != particle){
            Vector3 displacement = particle->GetPosition() - registeration.particle->GetPosition();
            real denominator = displacement.Magnitude() * displacement.Magnitude() * displacement.Magnitude();
            Vector3 force = displacement * (-1 * gravitationalConstant * particle->GetMass() * registeration.particle->GetMass() /  denominator);
            particle->AddForce(force);
        }
    }
}

IPhysics::ParticleSpring::ParticleSpring(Particle* otherParticle, real springConstant, real restLength){
    otherParticle = otherParticle;
    springConstant = springConstant;
    restLength = restLength;
}

void IPhysics::ParticleSpring::UpdateForce(Particle* particle, real duration){
    Vector3 direction;
    Vector3 force;
    direction = particle->GetPosition() - otherParticle->GetPosition();
    real magnitude = direction.Magnitude();
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    particle->AddForce(force);

}

IPhysics::ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 anchoredPosition, real springConstant, real restLength){
    anchoredPosition = anchoredPosition;
    springConstant = springConstant;
    restLength = restLength;
}

void IPhysics::ParticleAnchoredSpring::UpdateForce(Particle* particle, real duration){
    Vector3 direction;
    Vector3 force;
    direction = particle->GetPosition() - anchoredPosition;
    real magnitude = direction.Magnitude();
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    particle->AddForce(force);

}

IPhysics::ParticleBungee::ParticleBungee(Particle* otherParticle, real springConstant, real restLength){
    otherParticle = otherParticle;
    springConstant = springConstant;
    restLength = restLength;
}

void IPhysics::ParticleBungee::UpdateForce(Particle* particle, real duration){
    Vector3 direction;
    Vector3 force;
    direction = particle->GetPosition() - otherParticle->GetPosition();
    real magnitude = direction.Magnitude();
    if (magnitude <= restLength){
        return;
    }
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    particle->AddForce(force);

}

IPhysics::ParticleAnchoredBungee::ParticleAnchoredBungee(Vector3 anchoredPosition, real springConstant, real restLength){
    anchoredPosition = anchoredPosition;
    springConstant = springConstant;
    restLength = restLength;
}

void IPhysics::ParticleAnchoredBungee::UpdateForce(Particle* particle, real duration){
    Vector3 direction;
    Vector3 force;
    direction = particle->GetPosition() - anchoredPosition;
    real magnitude = direction.Magnitude();
    if (magnitude <= restLength){
        return;
    }
    force = direction;
    force.Normalise();
    force = force * -1 * springConstant * (magnitude - restLength);
    particle->AddForce(force);

}

IPhysics::ParticleBuoyancy::ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity){
    maxDepth = maxDepth;
    volume = volume;
    waterHeight = waterHeight;
    liquidDensity = liquidDensity;
}

void IPhysics::ParticleBuoyancy::UpdateForce(Particle* particle, real duration){
    real depth = particle->GetPosition().GetY();
    if (depth <= waterHeight - maxDepth){
        particle->AddForce(Vector3(0, volume * liquidDensity, 0));
    }
    else if (depth >= waterHeight + maxDepth){
        return;
    }
    else{
        particle->AddForce(Vector3(0, ((depth - maxDepth - waterHeight) / 2 * maxDepth) * volume * liquidDensity, 0));
    }
}

IPhysics::ParticleFakeAnchoredSpring::ParticleFakeAnchoredSpring(Vector3 anchoredPosition, real springConstant, real damping){
    anchoredPosition = anchoredPosition;
    springConstant = springConstant;
    damping = damping;
}

void IPhysics::ParticleFakeAnchoredSpring::UpdateForce(Particle* particle, real duration){
    Vector3 position = particle->GetPosition() - anchoredPosition;
    real gamma = 0.5f * RealSqrt(4 * springConstant - damping * damping);
    if (!gamma){
        return;
    }
    Vector3 c = position * (damping / ( 2.0f * gamma)) + particle->GetVelocity() * (1.0f / gamma);

    Vector3 target = position * RealCos(gamma * duration) + c * RealSin(gamma * duration);
    target *= RealExp(-0.5f * duration * damping);

    Vector3 acceleration = (target - position) * ( 1.0f / duration * duration ) - particle->GetVelocity() * duration;
    particle->AddForce(acceleration * particle->GetMass());
}