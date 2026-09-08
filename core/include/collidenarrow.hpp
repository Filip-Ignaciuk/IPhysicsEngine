#ifndef IPHYSICS_COLLIDENARROW_HPP
#define IPHYSICS_COLLIDENARROW_HPP
#include "components/primitive.hpp"
#include "contacts.hpp"

namespace IPhysics {
struct CollisionData {
  Contact* contactsArray;
  Contact* contacts;
  int contactsLeft;
  unsigned contactCount;
  real friction;
  real restitution;
  real tolerance;

  void AddContacts(unsigned count);
};

// Quick tests to allow the CollisionDetector class to exit early in case they
// are not colliding.
class IntersectionTests {
 public:
  static real TransformToAxis(const CollisionBox& box, const Vector3& axis);
  static bool OverlapOnAxis(const CollisionBox& box1, const CollisionBox& box2,
                            const Vector3& axis, const Vector3& toCentre);
  static bool BoxAndHalfSpace(const CollisionBox& box,
                              const CollisionPlane& plane);
  static bool BoxAndBox(const CollisionBox& box1, const CollisionBox& box2);
};

class CollisionDetector {
 public:
  static unsigned SphereAndSphere(const CollisionSphere& firstSphere,
                                  const CollisionSphere& secondSphere,
                                  CollisionData* data);
  static unsigned SphereAndHalfSpace(const CollisionSphere& sphere,
                                     const CollisionPlane& plane,
                                     CollisionData* data);
  static unsigned SphereAndPlane(const CollisionSphere& sphere,
                                 const CollisionPlane& plane,
                                 CollisionData* data);
  static unsigned BoxAndHalfSpace(const CollisionBox& box,
                                  const CollisionPlane& plane,
                                  CollisionData* data);
  static unsigned BoxAndPlane(const CollisionBox& box,
                              const CollisionPlane& plane, CollisionData* data);
  static unsigned BoxAndSphere(const CollisionBox& box,
                               const CollisionSphere& sphere,
                               CollisionData* data);
  static unsigned BoxAndBox(const CollisionBox& box1, const CollisionBox& box2,
                            CollisionData* data);

 private:
  static bool TryAxis(const CollisionBox& box1, const CollisionBox& box2,
                      Vector3 axis, const Vector3& toCentre, unsigned index,
                      real& smallestPenetration, unsigned& smallestCase);
  static real PenetrationOnAxis(const CollisionBox& box1,
                                const CollisionBox& box2, const Vector3& axis,
                                const Vector3& toCentre);
  static void FillPointFaceBoxBox(const CollisionBox& box1,
                                  const CollisionBox& box2,
                                  const Vector3& toCentre, CollisionData* data,
                                  unsigned best, real penetration);
  static Vector3 ContactPoint(const Vector3& pointOnOneEdge,
                              const Vector3& oneAxis, real oneSize,
                              const Vector3& pointOnTwoEdge,
                              const Vector3& twoAxis, real twoSize,
                              bool useOne);
};
}  // namespace IPhysics

#endif