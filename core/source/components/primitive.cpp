#include "components/primitive.hpp"

IPhysics::Vector3 IPhysics::CollisionPrimitive::GetAxis(unsigned _index) const{
    return transform.GetAxisVector(_index);
}
