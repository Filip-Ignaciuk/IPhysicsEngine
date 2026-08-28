#include "downwardgravity.hpp"

// Constructors
IPhysics::DownwardGravity::DownwardGravity(const Vector3& _gravity){
    m_gravity = _gravity;
}

// Mutators
void IPhysics::DownwardGravity::UpdateForce(RigidBody* _rigidBody, real _duration){
    if(!_rigidBody->HasFiniteMass()){
        return;
    }
    _rigidBody->AddForce(m_gravity * _rigidBody->GetMass());
}