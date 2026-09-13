#ifndef IPHYSICS_BARNESHUTGRAVITY_HPP
#define IPHYSICS_BARNESHUTGRAVITY_HPP

#include "bhtn.hpp"
#include "gravity.hpp"

namespace IPhysics {
class BarnesHutGravity : public Gravity {
 public:
  // Constructors
  explicit BarnesHutGravity(IPhysics::real gravityConstant,
                   IPhysics::real thresholdValue);

  // Mutators
  void AddObject(IPhysics::Object* object) override;
  void RemoveObject(IPhysics::Object* object) override;

  void UpdateForce(IPhysics::RigidBody* rigidBody,
                   IPhysics::real duration) override;

 private:
  int totalProcessedParticles = 0;

  bhtn* root = nullptr;
  IPhysics::real thresholdValue;
  IPhysics::real minWidth = 0.001;

  void CreateTree();
  void CreateTreeRoot();

  static void AddObjectToNode(bhtn* node, IPhysics::RigidBody* rigidBody);

  [[nodiscard]] IPhysics::Vector3 CalculateGravityForce(
      IPhysics::real mass1, const IPhysics::Vector3& centreOfMass1,
      IPhysics::real mass2, const IPhysics::Vector3& centreOfMass2) const;

  IPhysics::Vector3 TraverseNode(const bhtn* node,
                                 const IPhysics::RigidBody* rigidBody);
};
}  // namespace IPhysics

#endif
