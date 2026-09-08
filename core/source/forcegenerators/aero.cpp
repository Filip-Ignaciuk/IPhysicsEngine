#include "aero.hpp"

// Constructors
IPhysics::Aero::Aero(const Matrix3& tensor,
    const Vector3& localPosition,
    const Vector3* windSpeed) :
    m_tensor(tensor),
    m_localPosition(localPosition),
    m_windSpeed(windSpeed)
{
}

// Mutators
void IPhysics::Aero::UpdateForce(RigidBody* rigidBody, real duration){
    Aero::UpdateForceFromTensor(rigidBody, duration, m_tensor);
}

void IPhysics::Aero::UpdateForceFromTensor(RigidBody* body,
    real duration,
    const Matrix3& tensor) const {
    // Calculate total velocity from wind and body
    Vector3 velocity = body->GetVelocity();
    velocity += *m_windSpeed;

    // Calculate the velocity in body coordinates
    Vector3 bodyVelocity =
        body->GetTransformMatrix().TransformInverseDirection(velocity);

    // Calculate the force in body coordinates
    Vector3 bodyForce = m_tensor.Transform(bodyVelocity);
    Vector3 force = body->GetTransformMatrix().TransformDirection(bodyForce);

    body->AddForceAtBodyPoint(force, m_localPosition);
}

/*
 *  AeroControl Force Generator
 */

// Constructors
IPhysics::AeroControl::AeroControl(const Matrix3& base,
    const Matrix3& minimumTensor,
    const Matrix3& maximumTensor,
    const Vector3& localPosition,
    const Vector3* windSpeed) :
    Aero(base, localPosition, windSpeed),
    m_maxTensor(maximumTensor),
    m_minTensor(minimumTensor),
    m_controlSetting(0)

{
}

// Mutators
void IPhysics::AeroControl::SetControl(real value){
    m_controlSetting = value;
}

void IPhysics::AeroControl::UpdateForce(RigidBody* rigidBody, real duration){
    Matrix3 tensor = GetTensor();
    Aero::UpdateForceFromTensor(rigidBody, duration, tensor);
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