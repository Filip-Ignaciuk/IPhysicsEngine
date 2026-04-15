#include "collidebroad.hpp"

IPhysicsEngine::BoundingSphere::BoundingSphere(const Vector3& _centre, real _radius) : m_centre(_centre), m_radius(_radius){

}

IPhysicsEngine::BoundingSphere::BoundingSphere(const BoundingSphere& _one, const BoundingSphere& _two){
    Vector3 centreOffset = _two.m_centre - _one.m_centre;
    real distance = centreOffset.SquareMagnitude();
    real radiusDifference = _two.m_radius - _one.m_radius;
    
    // Check whether one sphere fully encompasses another.
    if (radiusDifference * radiusDifference >= distance){
        if(_one.m_radius > _two.m_radius){
            m_centre = _one.m_centre;
            m_radius = _one.m_radius;
        }
        else{
            m_centre = _two.m_centre;
            m_radius = _two.m_radius;
        }
    }
    else{
        distance = RealSqrt(distance);
        m_radius = (distance + _one.m_radius + _two.m_radius) * ((real)0.5);
        // We calculate the new centre based of the centre of one moved towards two's by an
        // Among proportional to two's radius.
        m_centre = _one.m_centre;
        if (distance > 0){
            m_centre += centreOffset * ((m_radius - _one.m_radius)/ distance);
        }
    }
}

bool IPhysicsEngine::BoundingSphere::Overlaps(const BoundingSphere* _other) const{
    Vector3 displacement = _other->m_centre - m_centre;
    real radiusLength = _other->m_radius + m_radius;
    return displacement.SquareMagnitude() < (radiusLength * radiusLength);
}

IPhysicsEngine::real IPhysicsEngine::BoundingSphere::GetSize() const{
    return ((real)1.333333) * R_PI * m_radius * m_radius * m_radius;
}