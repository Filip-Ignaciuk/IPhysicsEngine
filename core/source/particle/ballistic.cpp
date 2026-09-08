#include "ballistic.hpp"

IPhysics::BallisticParticle::BallisticParticle(Vector3 position, real damping,
                                               real inverseMass)
    : Particle(position, damping, inverseMass) {}

bool IPhysics::BallisticParticle::Integrate(real duration) {
  Particle::Integrate(duration);

  if (m_position.GetY() < 0) {
    return false;
  }
  return true;
}
