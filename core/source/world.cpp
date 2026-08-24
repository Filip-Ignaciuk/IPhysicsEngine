#include "world.hpp"

/*
 * World
 */

// Mutators
void IPhysics::World::StartFrame(){
    auto iterator = m_objects.begin();
    while (iterator != m_objects.end())
    {
        Object* object = *iterator;
        auto* rigidbody = object->GetComponent<RigidBody>();
        rigidbody->ClearAccumulators();
        rigidbody->CalculateDerivedData();
        ++iterator;
    }
}

void IPhysics::World::RunPhysics(real _timestep){
    // Update the forces
    m_registry.UpdateForces(_timestep);

    // Move the objects
    auto iterator = m_objects.begin();
    while(iterator != m_objects.end()){
        Object* object = *iterator;
        auto* rigidbody = object->GetComponent<RigidBody>();
        rigidbody->Integrate(_timestep);
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

void IPhysics::World::RemoveObject(Object* _object){
    RemoveForceRegistration(_object);
    m_objects.erase(
        std::ranges::remove(
            m_objects,
            _object).begin(),
            m_objects.end());
}

void IPhysics::World::RemoveLastObject() {
    m_objects.pop_back();
}

void IPhysics::World::AddForceRegistration(Object* _object,
                                           const std::shared_ptr<ForceGenerator>& _forceGenerator){
    m_registry.Add(_object, _forceGenerator);
}

void IPhysics::World::RemoveForceRegistration(Object* _object){
    m_registry.Remove(_object);
}

void IPhysics::World::SetPhysicsState(bool _state){
    m_physicsState = _state;
}

// Queries
const std::vector<IPhysics::Object*>& IPhysics::World::GetObjects()  {
    return m_objects;
}

const IPhysics::ForceRegistration& IPhysics::World::GetForceRegistration(
    Object* _object) const{
    return *m_registry.Get(_object);
}

const IPhysics::ForceRegistry& IPhysics::World::GetForceRegistry() const{
    return m_registry;
}

bool IPhysics::World::GetPhysicsState() const{
    return m_physicsState;
}