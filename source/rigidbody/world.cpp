#include "rigidbody/world.hpp"

void IPhysicsEngine::World::StartFrame(){
    Objects::iterator iterator = m_objects.begin();
    while (iterator != m_objects.end())
    {
        Object* object = *iterator;
        RigidBody* rigidbody = object->GetComponent<RigidBody>();
        rigidbody->ClearAccumulators();
        rigidbody->CalculateDerivedData();
        ++iterator;
    }
    
}

void IPhysicsEngine::World::RunPhysics(){
    m_registery.UpdateForces(m_timestep);
    Objects::iterator iterator = m_objects.begin();
    while(iterator != m_objects.end()){
        Object* object = *iterator;
        RigidBody* rigidbody = object->GetComponent<RigidBody>();
        rigidbody->Integrate(m_timestep);
        ++iterator;
    }
}

void IPhysicsEngine::World::AddObject(Object* _object){
    m_objects.emplace_back(_object);
}
        

void IPhysicsEngine::World::AddForceRegistry(Object* _object, ForceGenerator* _forceGenerator){
    m_registery.Add(_object->GetComponent<RigidBody>(), _forceGenerator);
}

void IPhysicsEngine::World::SetTimeStep(real _timestep){
    m_timestep = _timestep;
}

void IPhysicsEngine::World::SetPhysicsState(bool _state){
    m_physicsState = _state;
}

bool IPhysicsEngine::World::GetPhysicsState(){
    return m_physicsState;
}
        
IPhysicsEngine::real IPhysicsEngine::World::GetTimeStep(){
    return m_timestep;
}

IPhysicsEngine::ForceRegistry& IPhysicsEngine::World::GetParticleForceRegistry(){
    return m_registery;
}

IPhysicsEngine::World::Objects& IPhysicsEngine::World::GetObjects(){
    return m_objects;
}
