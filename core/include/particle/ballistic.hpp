#ifndef IPHYSICS_BALLISTIC_HPP
#define IPHYSICS_BALLISTIC_HPP
#include "particle.hpp"

namespace IPhysics {
class BallisticParticle : public Particle {
 public:
  BallisticParticle();
  BallisticParticle(Vector3 position, real damping, real inverseMass);
  bool Integrate(real duration) override;
};
}  // namespace IPhysics

#endif