#include "collidebroad.hpp"

IPhysics::BoundingSphere::BoundingSphere(const Vector3& centre, real radius)
    : m_centre(centre), m_radius(radius) {}

IPhysics::BoundingSphere::BoundingSphere(const BoundingSphere& one,
                                         const BoundingSphere& two) {
  Vector3 centreOffset = two.m_centre - one.m_centre;
  real distance = centreOffset.SquareMagnitude();
  real radiusDifference = two.m_radius - one.m_radius;

  // Check whether one sphere fully encompasses another.
  if (radiusDifference * radiusDifference >= distance) {
    if (one.m_radius > two.m_radius) {
      m_centre = one.m_centre;
      m_radius = one.m_radius;
    } else {
      m_centre = two.m_centre;
      m_radius = two.m_radius;
    }
  } else {
    distance = RealSqrt(distance);
    m_radius = (distance + one.m_radius + two.m_radius) * ((real)0.5);
    // We calculate the new centre based of the centre of one moved towards
    // two's by an Among proportional to two's radius.
    m_centre = one.m_centre;
    if (distance > 0) {
      m_centre += centreOffset * ((m_radius - one.m_radius) / distance);
    }
  }
}

bool IPhysics::BoundingSphere::Overlaps(const BoundingSphere* other) const {
  Vector3 displacement = other->m_centre - m_centre;
  real radiusLength = other->m_radius + m_radius;
  return displacement.SquareMagnitude() < (radiusLength * radiusLength);
}

IPhysics::real IPhysics::BoundingSphere::GetSize() const {
  return ((real)1.333333) * R_PI * m_radius * m_radius * m_radius;
}