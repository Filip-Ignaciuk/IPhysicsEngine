#include "world.hpp"

void IPhysicsEngine::World::StartFrame(){
    Rigidbodies::iterator rigidBodiesIterator = m_rigidBodies.begin();
    while (rigidBodiesIterator != m_rigidBodies.end())
    {
        RigidBody* rigidbody = *rigidBodiesIterator;
        rigidbody->ClearAccumulators();
        rigidbody->CalculateDerivedData();
        ++rigidBodiesIterator;
    }
    
}

void IPhysicsEngine::World::RunPhysics(real _duration){
    m_registery.UpdateForces(_duration);
    Rigidbodies::iterator iterator = m_rigidBodies.begin();
    while(iterator != m_rigidBodies.end()){
        RigidBody* rigidbody = *iterator;
        rigidbody->Integrate(_duration);
        ++iterator;
    }
}

IPhysicsEngine::ForceRegistry& IPhysicsEngine::World::GetParticleForceRegistry(){
    return m_registery;
}

IPhysicsEngine::World::Rigidbodies& IPhysicsEngine::World::GetRigidBodies(){
    return m_rigidBodies;
}
