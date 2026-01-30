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

IPhysicsEngine::Spring::Spring(const Vector3& _localConnectionPoint, RigidBody* _other, const Vector3& _otherLocalConnectionPoint, real _springConstant, real _restLength) : 
    m_localConnectionPoint(_localConnectionPoint),
    m_localOtherConnectionPoint(_otherLocalConnectionPoint),
    m_springConstant(_springConstant),
    m_restLength(_restLength)
{
}


void IPhysicsEngine::Spring::UpdateForce(RigidBody* _rigidBody, real _duration){
    Vector3 lws = _rigidBody->GetPointInWorldSpace(m_localConnectionPoint);
    Vector3 ows = _rigidBody->GetPointInWorldSpace(m_localOtherConnectionPoint);

    Vector3 force = lws - ows;

    real magnitude = force.Magnitude();

    magnitude = RealAbs(magnitude - m_restLength);
    magnitude *= m_springConstant;

    force.Normalise();
    force *= -magnitude;
    _rigidBody->AddForceAtPoint(force, lws);

}

IPhysicsEngine::Aero::Aero(const Matrix3& _tensor, const Vector3& _localPosition, const Vector3* _windspeed) :
    m_tensor(_tensor),
    m_localPosition(_localPosition),
    m_windspeed(_windspeed)
{
}

void IPhysicsEngine::Aero::UpdateForceFromTensor(RigidBody* _body, real _duration, const Matrix3& _tensor){
    // Calculate total velocity from wind and body
    Vector3 velocity = _body->GetVelocity();
    velocity += *m_windspeed;

    // Calculate the velocity in body coordinates
    Vector3 bodyVelocity = _body->GetTransformMatrix().TransformInverseDirection(velocity);
    
    // Calculate the force in body coordinates
    Vector3 bodyForce = m_tensor.Transform(bodyVelocity);
    Vector3 force = _body->GetTransformMatrix().TransformDirection(bodyForce);

    _body->AddForceAtBodyPoint(force, m_localPosition);
}

void IPhysicsEngine::Aero::UpdateForce(RigidBody* _rigidBody, real _duration){
    Aero::UpdateForceFromTensor(_rigidBody, _duration, m_tensor);
}

IPhysicsEngine::Matrix3 IPhysicsEngine::AeroControl::GetTensor(){
    // TO DO
    if (m_controlSetting <= -1.0f){
        return m_minTensor;
    }
    else if (m_controlSetting >= 1.0f){
        return m_maxTensor;
    }
    else if (m_controlSetting < 0.0f){
        return Matrix3::LinearInterpolate(m_minTensor, m_tensor, m_controlSetting + 1.0f);
    }
    else if (m_controlSetting > 0.0f){
        return Matrix3::LinearInterpolate(m_tensor, m_maxTensor, m_controlSetting);
    }
    else{
        return m_tensor;
    }
}

IPhysicsEngine::AeroControl::AeroControl(const Matrix3& _base, const Matrix3& _minimumTensor, const Matrix3& _maximumTensor, const Vector3& _localPosition, const Vector3* _windspeed) :
    Aero(_base, _localPosition, _windspeed),
    m_minTensor(_minimumTensor),
    m_maxTensor(_maximumTensor),
    m_controlSetting(0)

{
}

void IPhysicsEngine::AeroControl::SetControl(real _value){
    m_controlSetting = _value;
}

void IPhysicsEngine::AeroControl::UpdateForce(RigidBody* _rigidBody, real _duration){
    Matrix3 tensor = GetTensor();
    Aero::UpdateForceFromTensor(_rigidBody, _duration, tensor);
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
