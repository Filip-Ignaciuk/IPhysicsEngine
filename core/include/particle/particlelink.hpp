#ifndef IPHYSICS_PARTICLELINK_HPP
#define IPHYSICS_PARTICLELINK_HPP
#include "particlecontacts.hpp"

namespace IPhysics {
class ParticleLink : public ParticleContactGenerator {
 public:
  Particle* particles[2];

 protected:
  real CurrentLength() const;
  virtual unsigned AddContact(ParticleContact* contact, unsigned limit) = 0;
};

class ParticleCable : public ParticleLink {
 public:
  real maxLength;
  real restitution;
  virtual unsigned AddContact(ParticleContact* contact, unsigned limit) const;
};

class ParticleRod : public ParticleLink {
 public:
  real length;
  real CurrentLength() const;
  virtual unsigned AddContact(ParticleContact* contact, unsigned limit) const;
};
}  // namespace IPhysics

#endif