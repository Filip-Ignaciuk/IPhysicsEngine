#pragma once
#include "core.hpp"

namespace IPhysicsEngine{
    class Contact{
        public:
        RigidBody* body[2];
        real friction;
        real restitution;
        Vector3 contactPoint;
        Vector3 contactNormal;
        real penetration;
        void SetBodyData(RigidBody* _one, RigidBody* _two, real _friction, real _restitution);
    };
}