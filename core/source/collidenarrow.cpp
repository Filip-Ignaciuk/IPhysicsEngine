#include "collidenarrow.hpp"

void IPhysics::CollisionData::AddContacts(unsigned count) {
  contactsLeft = contactsLeft - count;
  contactCount = contactCount + count;
  contacts = contacts + count;
}

IPhysics::real IPhysics::IntersectionTests::TransformToAxis(
    const CollisionBox& box, const Vector3& axis) {
  return box.halfSize.x * RealAbs(axis * box.GetAxis(0)) +
         box.halfSize.y * RealAbs(axis * box.GetAxis(1)) +
         box.halfSize.z * RealAbs(axis * box.GetAxis(2));
}

bool IPhysics::IntersectionTests::OverlapOnAxis(const CollisionBox& box1,
                                                const CollisionBox& box2,
                                                const Vector3& axis,
                                                const Vector3& toCentre) {
  real projectOne = TransformToAxis(box1, axis);
  real projectTwo = TransformToAxis(box2, axis);
  real distance = RealAbs(toCentre * axis);
  return (distance < projectOne + projectTwo);
}
bool IPhysics::IntersectionTests::BoxAndHalfSpace(const CollisionBox& box,
                                                  const CollisionPlane& plane) {
  real projectedRadius = TransformToAxis(box, plane.normal);
  real boxDistance = plane.normal * box.GetAxis(3) - projectedRadius;
  return boxDistance <= plane.offset;
}

bool IPhysics::IntersectionTests::BoxAndBox(const CollisionBox& box1,
                                            const CollisionBox& box2) {
  Vector3 toCentre = box2.GetAxis(3) - box1.GetAxis(3);
  return OverlapOnAxis(box1, box2, box1.GetAxis(0), toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(1), toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(2), toCentre) &&
         OverlapOnAxis(box1, box2, box2.GetAxis(0), toCentre) &&
         OverlapOnAxis(box1, box2, box2.GetAxis(1), toCentre) &&
         OverlapOnAxis(box1, box2, box2.GetAxis(2), toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(0) % box2.GetAxis(0),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(0) % box2.GetAxis(1),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(0) % box2.GetAxis(2),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(1) % box2.GetAxis(0),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(1) % box2.GetAxis(1),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(1) % box2.GetAxis(2),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(2) % box2.GetAxis(0),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(2) % box2.GetAxis(1),
                       toCentre) &&
         OverlapOnAxis(box1, box2, box1.GetAxis(2) % box2.GetAxis(2), toCentre);
}

unsigned IPhysics::CollisionDetector::SphereAndSphere(
    const CollisionSphere& firstSphere, const CollisionSphere& secondSphere,
    CollisionData* data) {
  if (data->contactsLeft <= 0) {
    return 0;
  }
  Vector3 positionOne = firstSphere.GetAxis(3);
  Vector3 positionTwo = secondSphere.GetAxis(3);

  Vector3 midline = positionOne - positionTwo;
  real size = midline.Magnitude();

  // Check if the line is in desired range
  if (size <= (real)0.0 || size >= firstSphere.radius + secondSphere.radius) {
    return 0;
  }

  Vector3 normal = midline * ((real)1.0 / size);

  Contact* contact = data->contacts;
  contact->contactNormal = normal;
  contact->contactPoint = positionOne + midline * (real)0.5;
  contact->penetration = (firstSphere.radius + secondSphere.radius - size);
  contact->SetBodyData(firstSphere.rigidbody, secondSphere.rigidbody,
                       data->friction, data->restitution);
  data->AddContacts(1);
  return 1;
}

unsigned IPhysics::CollisionDetector::SphereAndHalfSpace(
    const CollisionSphere& sphere, const CollisionPlane& plane,
    CollisionData* data) {
  if (data->contactsLeft <= 0) {
    return 0;
  }
  Vector3 position = sphere.GetAxis(3);
  // Find the distance from the plane to the centre of the sphere
  real ballDistance = sphere.radius - (plane.normal * position) - plane.offset;
  // Ball is not penatrating the plane.
  if (ballDistance >= 0) {
    return 0;
  }

  Contact* contact = data->contacts;
  contact->contactNormal = plane.normal;
  contact->penetration = -ballDistance;
  contact->contactPoint =
      position - plane.normal * (ballDistance + sphere.radius);
  contact->SetBodyData(sphere.rigidbody, NULL, data->friction,
                       data->restitution);
  data->AddContacts(1);
  return 1;
}

unsigned IPhysics::CollisionDetector::SphereAndPlane(
    const CollisionSphere& sphere, const CollisionPlane& plane,
    CollisionData* data) {
  if (data->contactsLeft <= 0) {
    return 0;
  }
  Vector3 position = sphere.GetAxis(3);
  // Find the distance from the plane.
  real centreDistance = (plane.normal * position) - plane.offset;
  if (centreDistance * centreDistance > sphere.radius * sphere.radius) {
    return 0;
  }
  Vector3 normal = plane.normal;
  real penatration = -centreDistance;
  if (centreDistance < 0) {
    normal *= -1;
    penatration = -penatration;
  }
  penatration += sphere.radius;

  Contact* contact = data->contacts;
  contact->contactNormal = plane.normal;
  contact->penetration = penatration;
  contact->contactPoint = position - plane.normal * centreDistance;
  contact->SetBodyData(sphere.rigidbody, NULL, data->friction,
                       data->restitution);
  data->AddContacts(1);
  return 1;
}

unsigned IPhysics::CollisionDetector::BoxAndHalfSpace(
    const CollisionBox& box, const CollisionPlane& plane, CollisionData* data) {
  if (data->contactsLeft <= 0) {
    return 0;
  }

  if (!IntersectionTests::BoxAndHalfSpace(box, plane)) {
    return 0;
  }
  static real multiples[8][3] = {{1, 1, 1},   {-1, 1, 1},  {1, -1, 1},
                                 {-1, -1, 1}, {1, 1, -1},  {-1, 1, -1},
                                 {1, -1, -1}, {-1, -1, -1}};
  Contact* contact = data->contacts;
  unsigned contactsUsed = 0;
  for (unsigned i = 0; i < 8; ++i) {
    Vector3 vertexPosition(multiples[i][0], multiples[i][1], multiples[i][2]);
    vertexPosition.ComponentProductUpdate(box.halfSize);
    vertexPosition = box.transform.Transform(vertexPosition);

    real vertexDistance = vertexPosition * plane.normal;

    if (vertexDistance <= plane.offset) {
      // We have penetration.
      contact->contactPoint = plane.normal;
      contact->contactPoint *= (vertexDistance - plane.offset);
      contact->contactPoint += vertexPosition;
      contact->contactNormal = plane.normal;
      contact->penetration = plane.offset - vertexDistance;
      contact->SetBodyData(box.rigidbody, plane.rigidbody, data->friction,
                           data->restitution);
      contact++;
      contactsUsed++;
      if (contactsUsed == (unsigned)data->contactsLeft) {
        return contactsUsed;
      }
    }
  }

  data->AddContacts(contactsUsed);
  return contactsUsed;
}

unsigned IPhysics::CollisionDetector::BoxAndSphere(
    const CollisionBox& box, const CollisionSphere& sphere,
    CollisionData* data) {
  Vector3 centre = sphere.GetAxis(3);
  Vector3 relativeCentre = box.transform.TransformInverse(centre);

  if (RealAbs(relativeCentre.x) - sphere.radius > box.halfSize.x ||
      RealAbs(relativeCentre.y) - sphere.radius > box.halfSize.y ||
      RealAbs(relativeCentre.z) - sphere.radius > box.halfSize.z) {
    return 0;
  }

  Vector3 closestPoint(0, 0, 0);
  real distance;

  distance = relativeCentre.x;
  if (distance > box.halfSize.x) {
    distance = box.halfSize.x;
  }
  if (distance < -box.halfSize.x) {
    distance = -box.halfSize.x;
  }
  closestPoint.x = distance;

  distance = relativeCentre.y;
  if (distance > box.halfSize.y) {
    distance = box.halfSize.y;
  }
  if (distance < -box.halfSize.y) {
    distance = -box.halfSize.y;
  }
  closestPoint.y = distance;

  distance = relativeCentre.z;
  if (distance > box.halfSize.z) {
    distance = box.halfSize.z;
  }
  if (distance < -box.halfSize.z) {
    distance = -box.halfSize.z;
  }
  closestPoint.z = distance;

  distance = (closestPoint - relativeCentre).SquareMagnitude();
  if (distance > sphere.radius * sphere.radius) {
    return 0;
  }

  Vector3 closestPointInWorld = box.transform.Transform(closestPoint);
  Contact* contact = data->contacts;
  contact->contactNormal = (closestPointInWorld - centre);
  contact->contactNormal.Normalise();
  contact->penetration = sphere.radius - RealSqrt(distance);
  contact->SetBodyData(box.rigidbody, sphere.rigidbody, data->friction,
                       data->restitution);
  data->AddContacts(1);
  return 1;
}
// We use a macro to make our lives easier, this macro is from the book.
#define CHECK_OVERLAP(axis, index)                                        \
  if (!TryAxis(box1, box2, (axis), toCentre, (index), penetration, best)) \
    return 0;

unsigned IPhysics::CollisionDetector::BoxAndBox(const CollisionBox& box1,
                                                const CollisionBox& box2,
                                                CollisionData* data) {
  Vector3 toCentre = box2.GetAxis(3) - box1.GetAxis(3);
  real penetration = REAL_MAX;
  unsigned best = 0xffffff;

  CHECK_OVERLAP(box1.GetAxis(0), 0);
  CHECK_OVERLAP(box1.GetAxis(1), 1);
  CHECK_OVERLAP(box1.GetAxis(2), 2);

  CHECK_OVERLAP(box2.GetAxis(0), 3);
  CHECK_OVERLAP(box2.GetAxis(1), 4);
  CHECK_OVERLAP(box2.GetAxis(2), 5);

  unsigned bestSingleAxis = best;

  CHECK_OVERLAP(box1.GetAxis(0) % box2.GetAxis(0), 6);
  CHECK_OVERLAP(box1.GetAxis(0) % box2.GetAxis(1), 7);
  CHECK_OVERLAP(box1.GetAxis(0) % box2.GetAxis(2), 8);
  CHECK_OVERLAP(box1.GetAxis(1) % box2.GetAxis(0), 9);
  CHECK_OVERLAP(box1.GetAxis(1) % box2.GetAxis(1), 10);
  CHECK_OVERLAP(box1.GetAxis(1) % box2.GetAxis(2), 11);
  CHECK_OVERLAP(box1.GetAxis(2) % box2.GetAxis(0), 12);
  CHECK_OVERLAP(box1.GetAxis(2) % box2.GetAxis(1), 13);
  CHECK_OVERLAP(box1.GetAxis(2) % box2.GetAxis(2), 14);

  if (best < 3) {
    // Box 2 has vertex in face of Box 1
    FillPointFaceBoxBox(box1, box2, toCentre, data, best, penetration);
    data->AddContacts(1);
    return 1;
  } else if (best < 6) {
    // Box 1 has vertex in face of Box 2
    FillPointFaceBoxBox(box2, box1, toCentre * -1.0f, data, best - 3,
                        penetration);
    data->AddContacts(1);
    return 1;
  } else {
    best -= 6;
    unsigned oneAxisIndex = best / 3;
    unsigned twoAxisIndex = best % 3;
    Vector3 oneAxis = box1.GetAxis(oneAxisIndex);
    Vector3 twoAxis = box2.GetAxis(twoAxisIndex);
    Vector3 axis = oneAxis % twoAxis;
    axis.Normalise();

    // If box is not pointing to the other box then make it.
    if (axis * toCentre > 0) {
      axis = axis * -1.0f;
    }

    Vector3 pointOnOneEdge = box1.halfSize;
    Vector3 pointOnTwoEdge = box2.halfSize;
    for (unsigned i = 0; i < 3; i++) {
      if (i == oneAxisIndex) {
        pointOnOneEdge[i] = 0;
      } else if (box1.GetAxis(i) * axis > 0) {
        pointOnOneEdge[i] = -pointOnOneEdge[i];
      }

      if (i == twoAxisIndex) {
        pointOnTwoEdge[i] = 0;
      } else if (box2.GetAxis(i) * axis < 0) {
        pointOnTwoEdge[i] = -pointOnTwoEdge[i];
      }
    }

    pointOnOneEdge = box1.transform * pointOnOneEdge;
    pointOnTwoEdge = box2.transform * pointOnTwoEdge;

    Vector3 vertex = ContactPoint(
        pointOnOneEdge, oneAxis, box1.halfSize[oneAxisIndex], pointOnTwoEdge,
        twoAxis, box2.halfSize[twoAxisIndex], bestSingleAxis > 2);

    Contact* contact = data->contacts;

    contact->penetration = penetration;
    contact->contactNormal = axis;
    contact->contactPoint = vertex;
    contact->SetBodyData(box2.rigidbody, box2.rigidbody, data->friction,
                         data->restitution);
    data->AddContacts(1);
    return 1;
  }
  return 0;
}

bool IPhysics::CollisionDetector::TryAxis(const CollisionBox& box1,
                                          const CollisionBox& box2,
                                          Vector3 axis, const Vector3& toCentre,
                                          unsigned index,
                                          real& smallestPenetration,
                                          unsigned& smallestCase) {
  // Dont bother checking almost parallel axes.
  if (axis.SquareMagnitude() < 0.0001) {
    return true;
  }

  axis.Normalise();
  real penetration = PenetrationOnAxis(box1, box2, axis, toCentre);
  if (penetration < 0) {
    return false;
  }
  if (penetration < smallestPenetration) {
    smallestPenetration = penetration;
    smallestCase = index;
  }
  return true;
}

IPhysics::real IPhysics::CollisionDetector::PenetrationOnAxis(
    const CollisionBox& box1, const CollisionBox& box2, const Vector3& axis,
    const Vector3& toCentre) {
  real projectOne = IntersectionTests::TransformToAxis(box1, axis);
  real projectTwo = IntersectionTests::TransformToAxis(box2, axis);
  real distance = RealAbs(toCentre * axis);
  return projectOne + projectTwo - distance;
}

void IPhysics::CollisionDetector::FillPointFaceBoxBox(
    const CollisionBox& box1, const CollisionBox& box2, const Vector3& toCentre,
    CollisionData* data, unsigned best, real penetration) {
  // This method defaults with vertex from box 2 intersecting a fact from box 1.
  Contact* contact = data->contacts;
  Vector3 normal = box1.GetAxis(best);
  if (box1.GetAxis(best) * toCentre > 0) {
    normal = normal * -1.0f;
  }

  Vector3 vertex = box2.halfSize;
  if (box2.GetAxis(0) * normal < 0) vertex.x = -vertex.x;
  if (box2.GetAxis(1) * normal < 0) vertex.y = -vertex.y;
  if (box2.GetAxis(2) * normal < 0) vertex.z = -vertex.z;

  contact->contactNormal = normal;
  contact->penetration = penetration;
  contact->contactPoint = box2.transform * vertex;
  contact->SetBodyData(box1.rigidbody, box2.rigidbody, data->friction,
                       data->restitution);
}

IPhysics::Vector3 IPhysics::CollisionDetector::ContactPoint(
    const Vector3& pointOnOneEdge, const Vector3& oneAxis, real oneSize,
    const Vector3& pointOnTwoEdge, const Vector3& twoAxis, real twoSize,
    bool useOne) {
  Vector3 centreOffset = pointOnOneEdge - pointOnTwoEdge;
  real edge1LengthSquared = oneAxis.SquareMagnitude();
  real edge2LengthSquared = twoAxis.SquareMagnitude();
  real directionDot = oneAxis * twoAxis;

  real offsetAlongEdge1 = oneAxis * centreOffset;
  real offsetAlongEdge2 = twoAxis * centreOffset;

  real denominator =
      edge1LengthSquared * edge2LengthSquared - directionDot * directionDot;

  if (RealAbs(denominator) < 0.0001f) {
    if (useOne) {
      return pointOnOneEdge;
    } else {
      return pointOnTwoEdge;
    }
  }

  real edge1ClosestOffset = (directionDot * offsetAlongEdge2 -
                             edge2LengthSquared * offsetAlongEdge1) /
                            denominator;
  real edge2ClosestOffset = (edge1LengthSquared * offsetAlongEdge2 -
                             directionDot * offsetAlongEdge1) /
                            denominator;

  if (edge1ClosestOffset > oneSize || edge1ClosestOffset < -oneSize ||
      edge2ClosestOffset > twoSize || edge2ClosestOffset < -twoSize) {
    if (useOne) {
      return pointOnOneEdge;
    } else {
      return pointOnTwoEdge;
    }
  }

  Vector3 closestPointOnEdge1 = pointOnOneEdge + oneAxis * edge1ClosestOffset;
  Vector3 closestPointOnEdge2 = pointOnTwoEdge + twoAxis * edge2ClosestOffset;

  return (closestPointOnEdge1 + closestPointOnEdge2) * 0.5f;
}
