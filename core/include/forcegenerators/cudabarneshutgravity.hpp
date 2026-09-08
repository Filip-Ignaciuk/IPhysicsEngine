#ifndef IPHYSICS_CUDABARNESHUTGRAVITY_HPP
#define IPHYSICS_CUDABARNESHUTGRAVITY_HPP

#include "gravity.hpp"

namespace IPhysics {
class RealGravityBarnesHutCuda : public IPhysics::ForceGenerator {
 public:
  explicit RealGravityBarnesHutCuda(const IPhysics::real& gravityConstant);

  void AddObject(IPhysics::Object* object);
  void RemoveObject(IPhysics::Object* object);

  void UpdateForce(IPhysics::RigidBody* rigidBody,
                   IPhysics::real duration) override;

 private:
  void CreateTree();
};
}  // namespace IPhysics

#endif
