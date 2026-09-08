#ifndef IPHYSICS_FORCEGENERATOR_HPP
#define IPHYSICS_FORCEGENERATOR_HPP

#include "object.hpp"
#include "rigidbody.hpp"

namespace IPhysics {
class ForceGenerator {
 public:
  // Deconstructors
  virtual ~ForceGenerator() = default;

  // Mutators
  virtual void UpdateForce(RigidBody* rigidBody, real duration) = 0;
};

struct ForceRegistration {
  RigidBody* rigidBody;
  std::shared_ptr<ForceGenerator> forceGenerator;

  bool operator==(const ForceRegistration& other) const {
    return rigidBody == other.rigidBody &&
           forceGenerator == other.forceGenerator;
  }
};

class ForceRegistry {
 public:
  // Constructors
  ForceRegistry();

  // Mutators
  void Add(Object* object,
           const std::shared_ptr<ForceGenerator>& forceGenerator);
  void Remove(Object* object,
              const std::shared_ptr<ForceGenerator>& forceGenerator);
  void Remove(Object* object);
  void RemoveAll();
  void Clear();
  void UpdateForces(real duration);

  // Queries
  const ForceRegistration* Get(Object* object) const;

 protected:
  std::vector<ForceRegistration> registrations;
};
}  // namespace IPhysics

#endif