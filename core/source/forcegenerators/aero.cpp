#include "aero.hpp"

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