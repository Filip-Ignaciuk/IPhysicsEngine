#include "gravity.hpp"

// Constructors
IPhysics::Gravity::Gravity(const real& gravityConstant){
    m_gravityConstant = gravityConstant;
}

// Mutators
void IPhysics::Gravity::AddObject(Object* object){
    m_rigidBodies.emplace_back(object->GetComponent<RigidBody>());
}

void IPhysics::Gravity::RemoveObject(Object* object){
    std::erase(m_rigidBodies, object->GetComponent<RigidBody>());
}

void IPhysics::Gravity::UpdateForce(RigidBody* rigidBody, real duration){
    Vector3 totalForce(0, 0, 0);
    for(RigidBody* rigidbody : m_rigidBodies){
        if(rigidBody == rigidbody){
            continue;
        }
        real totalMass = rigidbody->GetMass() * rigidBody->GetMass();
        Vector3 distance = rigidBody->GetPosition() - rigidbody->GetPosition();
        real distanceMagnitude = distance.Magnitude();

        real forceMagnitude = -1
        * m_gravityConstant * totalMass
        / (distanceMagnitude * distanceMagnitude * distanceMagnitude);

        totalForce += distance * forceMagnitude;
    }
    rigidBody->AddForce(totalForce);
}