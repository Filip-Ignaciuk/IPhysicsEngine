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

void IPhysicsEngine::World::RunPhysics(real _duration){
    m_registery.UpdateForces(_duration);
    Objects::iterator iterator = m_objects.begin();
    while(iterator != m_objects.end()){
        Object* object = *iterator;
        RigidBody* rigidbody = object->GetComponent<RigidBody>();
        rigidbody->Integrate(_duration);
        ++iterator;
    }
}

void IPhysicsEngine::World::AddObject(Object* _object){
    m_objects.emplace_back(_object);
}
        

void IPhysicsEngine::World::AddForceRegistry(Object* _object, ForceGenerator* _forceGenerator){
    m_registery.Add(_object->GetComponent<RigidBody>(), _forceGenerator);
}

IPhysicsEngine::ForceRegistry& IPhysicsEngine::World::GetParticleForceRegistry(){
    return m_registery;
}

IPhysicsEngine::World::Objects& IPhysicsEngine::World::GetObjects(){
    return m_objects;
}
