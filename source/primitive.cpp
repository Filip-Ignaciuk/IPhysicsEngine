#include "primitive.hpp"

IPhysicsEngine::Vector3 IPhysicsEngine::CollisionPrimitive::GetAxis(unsigned _index) const{
    return transform.GetAxisVector(_index);
}
