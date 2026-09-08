#ifndef IPHYSICS_SPRING_HPP
#define IPHYSICS_SPRING_HPP

#include "core.hpp"
#include "forcegenerator.hpp"

namespace IPhysics {
class Spring : public ForceGenerator {
 public:
  // Constructors
  Spring(const Vector3& localConnectionPoint, RigidBody* other, const Vector3&,
         real springConstant, real restLength);

  // Mutators
  void UpdateForce(RigidBody* rigidBody, real duration) override;

 private:
  Vector3 m_localConnectionPoint;
  Vector3 m_localOtherConnectionPoint;
  IPhysics::RigidBody* m_other;
  IPhysics::real m_springConstant;
  real m_restLength;
};
}  // namespace IPhysics
#endif
