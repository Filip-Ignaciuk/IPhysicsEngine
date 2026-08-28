#include "forcegenerators/spring.hpp"

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