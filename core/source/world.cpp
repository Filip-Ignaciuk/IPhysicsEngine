#include "world.hpp"
#include "object.hpp"

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

}

void IPhysics::World::AddObject(Object* object) {
  m_objects.emplace_back(object);
}

void IPhysics::World::RemoveObject(Object* object) {
  RemoveForceRegistration(object);
  m_objects.erase(std::ranges::remove(m_objects, object).begin(),
                  m_objects.end());
  delete object;
}

void IPhysics::World::RemoveLastObject(){
  delete m_objects.back();
  m_objects.pop_back();
}

void IPhysics::World::RemoveAllObjects(){
  for(IPhysics::Object* object : m_objects){
    delete object;
  }
  m_objects.clear();
}

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

IPhysics::Object* IPhysics::World::GetLastObject(){
  return m_objects.back();
}

int IPhysics::World::GetNumberOfObjects() const{
  return m_objects.size();
}

const IPhysics::ForceRegistration& IPhysics::World::GetForceRegistration(
    Object* object) const {
  return *m_registry.Get(object);
}

const IPhysics::ForceRegistry& IPhysics::World::GetForceRegistry() const {
  return m_registry;
}

bool IPhysics::World::GetPhysicsState() const { return m_physicsState; }
