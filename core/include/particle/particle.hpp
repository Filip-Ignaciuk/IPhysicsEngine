#ifndef IPHYSICS_PARTICLE_HPP
#define IPHYSICS_PARTICLE_HPP
#include "core.hpp"

namespace IPhysics {
class Particle {
 protected:
  Vector3 m_position;
  Vector3 m_velocity;
  Vector3 m_acceleration;
  Vector3 m_forceAccumulated;
  // Simple and inaccurate form of drag.
  real m_damping;
  real m_inverseMass;

 public:
  Particle();
  Particle(Vector3 position, real damping, real inverseMass);

  void SetMass(real mass);
  void SetInverseMass(real inverseMass);
  void SetDamping(real damping);
  void SetPosition(Vector3 position);
  void SetVelocity(Vector3 velocity);
  void SetAcceleration(Vector3 acceleration);
  void AddForce(Vector3 force);
  virtual bool Integrate(real duration);
  void ClearAccumulator();

  real GetKineticEnergy() const;
  Vector3 GetPosition() const;
  Vector3 GetVelocity() const;
  Vector3 GetAcceleration() const;
  real GetDamping() const;
  real GetInverseMass() const;
  real GetMass() const;
};
}  // namespace IPhysics

#endif
