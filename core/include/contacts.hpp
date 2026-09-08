#ifndef IPHYSICS_CONTACTS_HPP
#define IPHYSICS_CONTACTS_HPP
#include "components/rigidbody.hpp"
#include "core.hpp"

namespace IPhysics {
class Contact {
  friend class ContactResolver;

 public:
  RigidBody* body[2];
  real friction;
  real restitution;
  Vector3 contactPoint;
  Vector3 contactNormal;
  real penetration;

 protected:
  Matrix3 contactToWorld;
  Vector3 contactVelocity;
  real desiredDeltaVelocity;
  Vector3 relativeContactPosition[2];

 public:
  void SetBodyData(RigidBody* one, RigidBody* two, real friction,
                   real restitution);

 protected:
  void MatchAwakeState();
  void CreateContactBasis();
  Vector3 CalculateFrictionlessImpulse(Matrix3* inverseInertiaTensor);
  void ApplyVelocityChange(Vector3 velocityChange[2],
                           Vector3 rotationChange[2]);
  void ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2],
                           real penetration);
  void CalculateInternals(real duration);
  Vector3 CalculateLocalVelocity(unsigned bodyIndex, real duration);
  void CalculateDesiredDeltaVelocity(real duration);
  void SwapBodies();
  void CalculateContactBasis();
};

class ContactResolver {
 protected:
  unsigned positionIterations;
  unsigned velocityIterations;
  real velocityEpsilon;
  real positionEpsilon;

 public:
  unsigned positionIterationsUsed;
  unsigned velocityIterationsUsed;
  void ResolveContacts(Contact* contactArray, unsigned numberOfContacts,
                       real duration);

 protected:
  void PrepareContacts(Contact* contactArray, unsigned numberOfContacts,
                       real duration);
  void AdjustVelocities(Contact* contactArray, unsigned numberOfContacts,
                        real duration);
  void AdjustPositions(Contact* contactArray, unsigned numberOfContacts,
                       real duration);
};
}  // namespace IPhysics

#endif