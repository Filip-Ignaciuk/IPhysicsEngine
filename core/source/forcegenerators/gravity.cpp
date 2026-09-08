#include "gravity.hpp"

// Constructors
IPhysics::Gravity::Gravity(const real& _gravityConstant){
    m_gravityConstant = _gravityConstant;
}

// Mutators
void IPhysics::Gravity::AddObject(Object* _object){
    m_rigidBodies.emplace_back(_object->GetComponent<RigidBody>());
}

void IPhysics::Gravity::RemoveObject(Object* _object){
    std::erase(m_rigidBodies, _object->GetComponent<RigidBody>());
}

void IPhysics::Gravity::UpdateForce(RigidBody* _rigidBody, real _duration){
    Vector3 totalForce(0, 0, 0);
    for(RigidBody* rigidbody : m_rigidBodies){
        if(_rigidBody == rigidbody){
            continue;
        }
        real totalMass = rigidbody->GetMass() * _rigidBody->GetMass();
        Vector3 distance = _rigidBody->GetPosition() - rigidbody->GetPosition();
        real distanceMagnitude = distance.Magnitude();

        real forceMagnitude = -1
        * m_gravityConstant * totalMass
        / (distanceMagnitude * distanceMagnitude * distanceMagnitude);

        totalForce += distance * forceMagnitude;
    }
    _rigidBody->AddForce(totalForce);
}