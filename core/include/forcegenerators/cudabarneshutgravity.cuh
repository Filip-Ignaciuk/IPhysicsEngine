#ifndef IPHYSICS_CUDABARNESHUTGRAVITY_CUH
#define IPHYSICS_CUDABARNESHUTGRAVITY_CUH

#include "gravity.hpp"

namespace IPhysics {

class CUDABarnesHutGravity : public Gravity {
 public:
  explicit CUDABarnesHutGravity(IPhysics::real gravityConstant,
                                IPhysics::real thresholdValue);

  void AddObject(IPhysics::Object* object) override;
  void RemoveObject(IPhysics::Object* object) override;

  void UpdateForce(IPhysics::RigidBody* rigidBody,
                   IPhysics::real duration) override;

 private:
  void CreateTree();

    IPhysics::real m_thresholdValue;

};
}

#endif
