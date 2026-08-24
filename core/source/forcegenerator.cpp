#include "forcegenerator.hpp"

/*
 *  Gravity Force Generator
 */

// Constructors
IPhysics::Gravity::Gravity(const Vector3& _gravity){
    m_gravity = _gravity;
}

// Mutators
void IPhysics::Gravity::UpdateForce(RigidBody* _rigidBody, real _duration){
    if(!_rigidBody->HasFiniteMass()){
        return;
    }

    _rigidBody->AddForce(m_gravity * _rigidBody->GetMass());

}

/*
 *  Real Gravity Force Generator
 */

// Constructors
IPhysics::RealGravity::RealGravity(const real& _gravityConstant){
    m_gravityConstant = _gravityConstant;
}

// Mutators
void IPhysics::RealGravity::AddObject(Object* _object){
    m_rigidBodies.emplace_back(_object->GetComponent<RigidBody>());
}

void IPhysics::RealGravity::RemoveObject(Object* _object){
    m_rigidBodies.erase(
        remove(
            m_rigidBodies.begin(),
            m_rigidBodies.end(),
            _object->GetComponent<RigidBody>()),
            m_rigidBodies.end());
}

void IPhysics::RealGravity::UpdateForce(RigidBody* _rigidBody, real _duration){
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

/*
 *  Spring Force Generator
 */

// Constructors
IPhysics::Spring::Spring(const Vector3& _localConnectionPoint,
    RigidBody* _other,
    const Vector3& _otherLocalConnectionPoint,
    real _springConstant,
    real _restLength) :
    m_localConnectionPoint(_localConnectionPoint),
    m_localOtherConnectionPoint(_otherLocalConnectionPoint),
    m_other(_other),
    m_springConstant(_springConstant),
    m_restLength(_restLength)
{
}

// Mutators
void IPhysics::Spring::UpdateForce(RigidBody* _rigidBody, real _duration){
    const Vector3 lws = _rigidBody->GetPointInWorldSpace(m_localConnectionPoint);
    const Vector3 ows = _rigidBody->GetPointInWorldSpace(m_localOtherConnectionPoint);

    Vector3 force = lws - ows;

    real magnitude = force.Magnitude();

    magnitude = RealAbs(magnitude - m_restLength);
    magnitude *= m_springConstant;

    force.Normalise();
    force *= -magnitude;
    _rigidBody->AddForceAtPoint(force, lws);

}

/*
 * Aero Force Generator
 */

// Constructors
IPhysics::Aero::Aero(const Matrix3& _tensor,
    const Vector3& _localPosition,
    const Vector3* _windSpeed) :
    m_tensor(_tensor),
    m_localPosition(_localPosition),
    m_windSpeed(_windSpeed)
{
}

// Mutators
void IPhysics::Aero::UpdateForce(RigidBody* _rigidBody, real _duration){
    Aero::UpdateForceFromTensor(_rigidBody, _duration, m_tensor);
}

void IPhysics::Aero::UpdateForceFromTensor(RigidBody* _body,
    real _duration,
    const Matrix3& _tensor) const {
    // Calculate total velocity from wind and body
    Vector3 velocity = _body->GetVelocity();
    velocity += *m_windSpeed;

    // Calculate the velocity in body coordinates
    Vector3 bodyVelocity =
        _body->GetTransformMatrix().TransformInverseDirection(velocity);
    
    // Calculate the force in body coordinates
    Vector3 bodyForce = m_tensor.Transform(bodyVelocity);
    Vector3 force = _body->GetTransformMatrix().TransformDirection(bodyForce);

    _body->AddForceAtBodyPoint(force, m_localPosition);
}

/*
 *  AeroControl Force Generator
 */

// Constructors
IPhysics::AeroControl::AeroControl(const Matrix3& _base,
    const Matrix3& _minimumTensor,
    const Matrix3& _maximumTensor,
    const Vector3& _localPosition,
    const Vector3* _windSpeed) :
    Aero(_base, _localPosition, _windSpeed),
    m_maxTensor(_maximumTensor),
    m_minTensor(_minimumTensor),
    m_controlSetting(0)

{
}

// Mutators
void IPhysics::AeroControl::SetControl(real _value){
    m_controlSetting = _value;
}

void IPhysics::AeroControl::UpdateForce(RigidBody* _rigidBody, real _duration){
    Matrix3 tensor = GetTensor();
    Aero::UpdateForceFromTensor(_rigidBody, _duration, tensor);
}

IPhysics::Matrix3 IPhysics::AeroControl::GetTensor(){
    if (m_controlSetting <= -1.0f){
        return m_minTensor;
    }
    else if (m_controlSetting >= 1.0f){
        return m_maxTensor;
    }
    else if (m_controlSetting < 0.0f){
        return Matrix3::LinearInterpolate(
            m_minTensor,
            m_tensor,
            m_controlSetting + 1.0f);
    }
    else if (m_controlSetting > 0.0f){
        return Matrix3::LinearInterpolate(
            m_tensor,
            m_maxTensor,
            m_controlSetting);
    }
    else{
        return m_tensor;
    }
}

IPhysics::ForceRegistry::ForceRegistry(){
    constexpr std::vector<ForceRegistration> temporary;
    registrations = temporary;
}

void IPhysics::ForceRegistry::Add(Object* _object, const std::shared_ptr<ForceGenerator>& _forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = _object->GetComponent<RigidBody>();
    forceRegistration.forceGenerator = _forceGenerator;
    registrations.emplace_back(forceRegistration);
}

void IPhysics::ForceRegistry::Remove(Object* _object, const std::shared_ptr<ForceGenerator>& _forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = _object->GetComponent<RigidBody>();
    forceRegistration.forceGenerator = _forceGenerator;
    registrations.erase(std::remove(registrations.begin(), registrations.end(), forceRegistration), registrations.end());
}

void IPhysics::ForceRegistry::Remove(Object* _object){
    auto* rigidBody = _object->GetComponent<RigidBody>();
    for(const ForceRegistration& forceRegistration : registrations){
        if(forceRegistration.rigidBody == rigidBody){
            registrations.erase(remove(registrations.begin(), registrations.end(), forceRegistration), registrations.end());
        }
    }
}

void IPhysics::ForceRegistry::RemoveAll() {
    registrations.clear();
}

IPhysics::ForceRegistration* IPhysics::ForceRegistry::Get(Object* _object) const {
    for (ForceRegistration forceRegistration : registrations) {
        if (forceRegistration.rigidBody == _object->GetComponent<RigidBody>()) {
            return &forceRegistration;
        }
    }
    return nullptr;
}

void IPhysics::ForceRegistry::Clear(){
    registrations.clear();
}

void IPhysics::ForceRegistry::UpdateForces(real _duration){
    std::vector<ForceRegistration>::iterator iterator = registrations.begin();

    while (iterator != registrations.end())
    {
        ForceRegistration forceRegistration = *iterator;
        forceRegistration.forceGenerator->UpdateForce(forceRegistration.rigidBody, _duration);
        ++iterator;
    }
}
