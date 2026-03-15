#include "contacts.hpp"
#include "rigidbody.hpp"

void IPhysicsEngine::Contact::SetBodyData(RigidBody* _one, RigidBody* _two, real _friction, real _restitution){
    body[0] = _one;
    body[1] = _two;
    friction = _friction;
    restitution = _restitution;
}
