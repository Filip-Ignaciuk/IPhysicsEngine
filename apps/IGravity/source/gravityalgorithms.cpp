#include "gravityalgorithms.hpp"

RealGravityCuda::RealGravityCuda(const IPhysics::real& _gravityConstant) {
    this->m_gravityConstant = _gravityConstant;
}

void RealGravityCuda::AddObject(IPhysics::Object *_object) {
    m_rigidBodies.emplace_back(_object->GetComponent<IPhysics::RigidBody>());
}

void RealGravityCuda::RemoveObject(IPhysics::Object *_object) {
    m_rigidBodies.erase(remove(m_rigidBodies.begin(), m_rigidBodies.end(), _object->GetComponent<IPhysics::RigidBody>()), m_rigidBodies.end());
}

void RealGravityCuda::UpdateForce(IPhysics::RigidBody *_rigidBody, IPhysics::real _duration) {
    IPhysics::Vector3 totalForce(0, 0, 0);
    for(IPhysics::RigidBody* rigidbody : m_rigidBodies){
        if(_rigidBody == rigidbody){
            continue;
        }
        IPhysics::real totalMass = rigidbody->GetMass() * _rigidBody->GetMass();
        IPhysics::Vector3 distance = _rigidBody->GetPosition() - rigidbody->GetPosition();
        IPhysics::real distanceMagnitude = distance.Magnitude();

        IPhysics::real forceMagnitude = -1 * m_gravityConstant * totalMass / (distanceMagnitude * distanceMagnitude * distanceMagnitude);
        totalForce += distance * forceMagnitude;
    }
    _rigidBody->AddForce(totalForce);
}
