#include "components/primitive.hpp"

IPhysics::Vector3 IPhysics::CollisionPrimitive::GetAxis(unsigned index) const {
  return transform.GetAxisVector(index);
}
