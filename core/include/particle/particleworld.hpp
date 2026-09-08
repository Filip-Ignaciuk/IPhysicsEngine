#ifndef IPHYSICS_PARTICLEWORLD_HPP
#define IPHYSICS_PARTICLEWORLD_HPP
#include <vector>

#include "particle.hpp"
#include "particleforcegenerator.hpp"
#include "particlelink.hpp"

namespace IPhysics {
class ParticleWorld {
 public:
  typedef std::vector<Particle*> Particles;
  typedef std::vector<ParticleContactGenerator*> ContactGenerators;

  ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

  void StartFrame();

  unsigned GenerateContacts();

  void Integrate(real duration);

  void RunPhysics(real duration);

  Particles& GetParticles();

  ContactGenerators& GetParticleContactGenerator();

  ParticleForceRegistry& GetParticleForceRegistry();

 protected:
  bool calculateIterations;

  Particles particles;

  ParticleForceRegistry particleForceRegistry;
  ParticleContactResolver particleContactResolvers;

  ContactGenerators contactGenerators;

  ParticleContact* contacts;

  unsigned maxContacts;
};

class ParticleGroundContactGenerator
    : public IPhysics::ParticleContactGenerator {
 private:
  std::vector<IPhysics::Particle*>* particles;
  real restitution;

 public:
  void Init(std::vector<IPhysics::Particle*>* particles, real restitution);
  virtual unsigned AddContact(ParticleContact* contact, unsigned limit) const;
};
}  // namespace IPhysics

#endif
