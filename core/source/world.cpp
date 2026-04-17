#include "world.hpp"


void IPhysics::World::StartFrame(){
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

void IPhysics::World::RunPhysics(){
    // Update the forces
    m_registery.UpdateForces(m_timestep);

    // Move the objects
    Objects::iterator iterator = m_objects.begin();
    while(iterator != m_objects.end()){
        Object* object = *iterator;
        RigidBody* rigidbody = object->GetComponent<RigidBody>();
        rigidbody->Integrate(m_timestep);
        ++iterator;
    }

    /*
    // Detect for collisions broad.
    PotentialContact contacts[MAX_CONTACTS];
    unsigned contactCount = m_root->GetPotentialContacts(contacts, MAX_CONTACTS);

    // Detect for collisions narrow
    CollisionData collisionData;
    Contact contactArray[MAX_CONTACTS];
    collisionData.contactsArray = contactArray;
    collisionData.contacts = contactArray;
    collisionData.contactsLeft = MAX_CONTACTS;
    collisionData.contactCount = 0;
    collisionData.friction = (real)0.2f;
    collisionData.restitution = (real)0.6f;
    collisionData.tolerance = (real)0.2f;

    for(int i = 0; i < contactCount; ++i){
        Object* firstBody = contacts[i].object[0];
        Object* secondBody = contacts[i].object[1];

        CollisionPrimitive* firstPrimitive = firstBody->GetComponent<CollisionPrimitive>();
        CollisionPrimitive* secondPrimitive = secondBody->GetComponent<CollisionPrimitive>();

        if(firstPrimitive->type == CollisionPrimitiveType::SPHERE && secondPrimitive->type == CollisionPrimitiveType::SPHERE){
            CollisionSphere* firstCollisionSphere = static_cast<CollisionSphere*>(firstPrimitive);
            CollisionSphere* secondCollisionSphere = static_cast<CollisionSphere*>(secondPrimitive);
            
            CollisionDetector::SphereAndSphere(*firstCollisionSphere, *secondCollisionSphere, &collisionData);
        }
    }

    // Resolve any detected Collisions.
    m_contactResolver.ResolveContacts(collisionData.contactsArray, collisionData.contactCount, m_timestep);
    */
}

void IPhysics::World::AddObject(Object* _object){
    m_objects.emplace_back(_object);
    /*
    RigidBody* body = _object->GetComponent<RigidBody>();
    const BoundingSphere boundingSphere(body->GetPosition(), body->GetMaxDistanceFromCentre());
    if(m_root == nullptr){
        m_root = new BoundingVolumeHierarchyNode<BoundingSphere>(nullptr, boundingSphere, _object);
    }
    else{
        m_root->Insert(_object, boundingSphere);
    }
    */
}
        

void IPhysics::World::AddForceRegistry(Object* _object, ForceGenerator* _forceGenerator){
    m_registery.Add(_object, _forceGenerator);
}

void IPhysics::World::SetTimeStep(real _timestep){
    m_timestep = _timestep;
}

void IPhysics::World::SetPhysicsState(bool _state){
    m_physicsState = _state;
}

bool IPhysics::World::GetPhysicsState(){
    return m_physicsState;
}
        
IPhysics::real IPhysics::World::GetTimeStep(){
    return m_timestep;
}

IPhysics::ForceRegistry& IPhysics::World::GetParticleForceRegistry(){
    return m_registery;
}

IPhysics::World::Objects& IPhysics::World::GetObjects(){
    return m_objects;
}

void IPhysics::World::RemoveObject(Object* _object){
    m_objects.erase(remove(m_objects.begin(), m_objects.end(), _object), m_objects.end());
}