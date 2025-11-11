#include "forcegenerator.hpp"
#include <algorithm>

IPhysicsEngine::Gravity::Gravity(const Vector3& _gravity){
    m_gravity = _gravity;
}

void IPhysicsEngine::Gravity::UpdateForce(RigidBody* _rigidBody, real _duration){
    if(!_rigidBody->HasFiniteMass()){
        return;
    }

    _rigidBody->AddForce(m_gravity * _rigidBody->GetMass());

}

IPhysicsEngine::Spring::Spring(const Vector3& _localConnectionPoint, RigidBody* _other, const Vector3& _otherConnectionPoint, real _springConstant, real _restLength) : 
    m_connectionPoint(_localConnectionPoint),
    m_otherConnectionPoint(_otherConnectionPoint),
    m_springConstant(_springConstant),
    m_restLength(_restLength)
{
}


void IPhysicsEngine::Spring::UpdateForce(RigidBody* _rigidBody, real _duration){
    Vector3 lws = _rigidBody->GetPointInWorldSpace(m_connectionPoint);
    Vector3 ows = _rigidBody->GetPointInWorldSpace(m_otherConnectionPoint);

    Vector3 force = lws - ows;

    real magnitude = force.Magnitude();

    magnitude = RealAbs(magnitude - m_restLength);
    magnitude *= m_springConstant;

    force.Normalise();
    force *= -magnitude;
    _rigidBody->AddForceAtPoint(force, lws);

}

void IPhysicsEngine::Aero::UpdateForce(RigidBody* _rigidBody, real _duration){
    Aero::UpdateForceFromTensor(_rigidBody, _duration, tensor);
}

void IPhysicsEngine::Aero::UpdateForceFromTensor(RigidBody* _body, real _duration, const Matrix3& _tensor){
    
}

IPhysicsEngine::ForceRegistry::ForceRegistry(){
    std::vector<ForceRegistration> temporary;
    registrations = temporary;
}

void IPhysicsEngine::ForceRegistry::Add(RigidBody* _rigidbody, ForceGenerator* _forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = _rigidbody;
    forceRegistration.forceGenerator = _forceGenerator;
    registrations.emplace_back(forceRegistration);
}

void IPhysicsEngine::ForceRegistry::Remove(RigidBody* _rigidbody, ForceGenerator* _forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = _rigidbody;
    forceRegistration.forceGenerator = _forceGenerator;
    registrations.erase(std::remove(registrations.begin(), registrations.end(), forceRegistration), registrations.end());
}
void IPhysicsEngine::ForceRegistry::Clear(){
    registrations.clear();
}
void IPhysicsEngine::ForceRegistry::UpdateForces(real _duration){
    std::vector<ForceRegistration>::iterator iterator = registrations.begin();
    while (iterator != registrations.end())
    {
        ForceRegistration forceRegisteration = *iterator;
        forceRegisteration.forceGenerator->UpdateForce(forceRegisteration.rigidBody, _duration);
        ++iterator;
    }
    
}
