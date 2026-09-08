#include "world.hpp"

/*
 * World
 */

// Mutators
void IPhysics::World::StartFrame() {
  auto iterator = m_objects.begin();
  while (iterator != m_objects.end()) {
    Object* object = *iterator;
    auto* rigidbody = object->GetComponent<RigidBody>();
    rigidbody->ClearAccumulators();
    rigidbody->CalculateDerivedData();
    ++iterator;
  }
}

void IPhysics::World::RunPhysics(real timestep) {
  // Update the forces
  m_registry.UpdateForces(timestep);

  // Move the objects
  auto iterator = m_objects.begin();
  while (iterator != m_objects.end()) {
    Object* object = *iterator;
    auto* rigidbody = object->GetComponent<RigidBody>();
    rigidbody->Integrate(timestep);
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

      CollisionPrimitive* firstPrimitive =
  firstBody->GetComponent<CollisionPrimitive>(); CollisionPrimitive*
  secondPrimitive = secondBody->GetComponent<CollisionPrimitive>();

      if(firstPrimitive->type == CollisionPrimitiveType::SPHERE &&
  secondPrimitive->type == CollisionPrimitiveType::SPHERE){ CollisionSphere*
  firstCollisionSphere = static_cast<CollisionSphere*>(firstPrimitive);
          CollisionSphere* secondCollisionSphere =
  static_cast<CollisionSphere*>(secondPrimitive);

          CollisionDetector::SphereAndSphere(*firstCollisionSphere,
  *secondCollisionSphere, &collisionData);
      }
  }

  // Resolve any detected Collisions.
  m_contactResolver.ResolveContacts(collisionData.contactsArray,
  collisionData.contactCount, m_timestep);
  */
}

void IPhysics::World::AddObject(Object* object) {
  m_objects.emplace_back(object);
  /*
  RigidBody* body = object->GetComponent<RigidBody>();
  const BoundingSphere boundingSphere(body->GetPosition(),
  body->GetMaxDistanceFromCentre()); if(m_root == nullptr){ m_root = new
  BoundingVolumeHierarchyNode<BoundingSphere>(nullptr, boundingSphere, object);
  }
  else{
      m_root->Insert(object, boundingSphere);
  }
  */
}

void IPhysics::World::RemoveObject(Object* object) {
  RemoveForceRegistration(object);
  m_objects.erase(std::ranges::remove(m_objects, object).begin(),
                  m_objects.end());
}

void IPhysics::World::RemoveLastObject() { m_objects.pop_back(); }

void IPhysics::World::AddForceRegistration(
    Object* object, const std::shared_ptr<ForceGenerator>& forceGenerator) {
  m_registry.Add(object, forceGenerator);
}

void IPhysics::World::RemoveForceRegistration(Object* object) {
  m_registry.Remove(object);
}

void IPhysics::World::SetPhysicsState(bool state) { m_physicsState = state; }

// Queries
const std::vector<IPhysics::Object*>& IPhysics::World::GetObjects() {
  return m_objects;
}

const IPhysics::ForceRegistration& IPhysics::World::GetForceRegistration(
    Object* object) const {
  return *m_registry.Get(object);
}

const IPhysics::ForceRegistry& IPhysics::World::GetForceRegistry() const {
  return m_registry;
}

bool IPhysics::World::GetPhysicsState() const { return m_physicsState; }