#include "downwardgravity.hpp"

// Constructors
IPhysics::DownwardGravity::DownwardGravity(const Vector3& gravity) {
  m_gravity = gravity;
}

// Mutators
void IPhysics::DownwardGravity::UpdateForce(RigidBody* rigidBody,
                                            real duration) {
  if (!rigidBody->HasFiniteMass()) {
    return;
  }
  rigidBody->AddForce(m_gravity * rigidBody->GetMass());
}