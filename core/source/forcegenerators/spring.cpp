#include "forcegenerators/spring.hpp"

// Constructors
IPhysics::Spring::Spring(const Vector3& localConnectionPoint,
    RigidBody* other,
    const Vector3& otherLocalConnectionPoint,
    real springConstant,
    real restLength) :
    m_localConnectionPoint(localConnectionPoint),
    m_localOtherConnectionPoint(otherLocalConnectionPoint),
    m_other(other),
    m_springConstant(springConstant),
    m_restLength(restLength)
{
}

// Mutators
void IPhysics::Spring::UpdateForce(RigidBody* rigidBody, real duration){
    const Vector3 lws = rigidBody->GetPointInWorldSpace(m_localConnectionPoint);
    const Vector3 ows = rigidBody->GetPointInWorldSpace(m_localOtherConnectionPoint);

    Vector3 force = lws - ows;

    real magnitude = force.Magnitude();

    magnitude = RealAbs(magnitude - m_restLength);
    magnitude *= m_springConstant;

    force.Normalise();
    force *= -magnitude;
    rigidBody->AddForceAtPoint(force, lws);

}