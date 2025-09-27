#include "particlecontacts.hpp"
#include "particleworld.hpp"

void IPhysicsEngine::ParticleContact::Resolve(real _duration){
    ResolveVelocity(_duration);
}

IPhysicsEngine::real IPhysicsEngine::ParticleContact::CalculateSeparatingVelocity() const{
    Vector3 relativeVelocity = particles[0]->GetVelocity();
    if (particles[1]){
        relativeVelocity -= particles[1]->GetVelocity();
    }
    return relativeVelocity * contactNormal;
}

void IPhysicsEngine::ParticleContact::ResolveVelocity(real _duration){
    real separatingVelocity = CalculateSeparatingVelocity();
    if (separatingVelocity > 0){
        return;
    }

    real newSeparatingVelocity = -separatingVelocity * restitution;
    
    Vector3 accelerationCausedVelocity = particles[0]->GetAcceleration();
    if (particles[1]){
        accelerationCausedVelocity -= particles[1]->GetAcceleration();
    }
    real accelerationCausedSeparatedVelocity = accelerationCausedVelocity * contactNormal * _duration;
    if (accelerationCausedSeparatedVelocity < 0){
        newSeparatingVelocity += restitution * accelerationCausedSeparatedVelocity;
        if (newSeparatingVelocity < 0){
            newSeparatingVelocity = 0;
        }
    }
    real deltaVelocity = newSeparatingVelocity - separatingVelocity;

    real totalInverseMass = particles[0]->GetInverseMass();
    if (particles[1]){
        totalInverseMass += particles[1]->GetInverseMass();
    }

    if (totalInverseMass <= 0){
        return;
    }

    real impulse = deltaVelocity / totalInverseMass;
    Vector3 impulsePerInverseMass = contactNormal * impulse;
    particles[0]->SetVelocity(particles[0]->GetVelocity() + impulsePerInverseMass * particles[0]->GetInverseMass());
    if (particles[1]){
        particles[1]->SetVelocity(particles[1]->GetVelocity() + impulsePerInverseMass * -particles[1]->GetInverseMass());
    }

}

void IPhysicsEngine::ParticleContact::ResolveInterpretation(real _duration){
    if (penetration <= 0){
        return;
    }

    real totalInverseMass = particles[0]->GetInverseMass();
    if (particles[1]){
        totalInverseMass += particles[1]->GetInverseMass();
    }

    if (totalInverseMass <= 0){
        return;
    }

    Vector3 movePerInverseMass = contactNormal * ( penetration / totalInverseMass );

    particleMovement[0] = movePerInverseMass * particles[0]->GetInverseMass();
    if (particles[1]){
        particleMovement[1] = movePerInverseMass * -particles[1]->GetInverseMass();
    }
    else{
        particleMovement->Clear();
    }

    particles[0]->SetPosition(particles[0]->GetPosition() + particleMovement[0]);

    if (particles[1]){
        particles[1]->SetPosition(particles[1]->GetPosition() + particleMovement[1]);
    }
}

IPhysicsEngine::ParticleContactResolver::ParticleContactResolver(unsigned _iterations) : iterations(_iterations){

}

void IPhysicsEngine::ParticleContactResolver::SetIterations(unsigned _iterations){
    iterations = _iterations;
}

void IPhysicsEngine::ParticleContactResolver::ResolveContacts(ParticleContact* _contactArray, unsigned _numberOfContacts, real _duration){
    unsigned i;
    iterationsUsed = 0;

    while (iterationsUsed < iterations){
        real max = Real_Max;
        unsigned maxIndex = _numberOfContacts;
        for (int i = 0; i < _numberOfContacts; i++)
        {
            real separateVelocity = _contactArray[i].CalculateSeparatingVelocity();
            if ( separateVelocity > max && separateVelocity < 0 || _contactArray[i].penetration > 0){
                max = separateVelocity;
                maxIndex = i;
            }
        }

        if (maxIndex == _numberOfContacts){
            break;
        }

        _contactArray[maxIndex].Resolve(_duration);

        iterationsUsed++;
        
    }
}

