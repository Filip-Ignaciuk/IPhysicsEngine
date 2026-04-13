#pragma once
#include "rigidbody/rigidbody.hpp"

namespace IPhysicsEngine
{
    class CollisionPrimitive{
        public:
        RigidBody* rigidbody;
        Matrix4 offset;
        Matrix4 transform;
        Vector3 GetAxis(unsigned _index) const;
    };

    class CollisionSphere : public CollisionPrimitive{
        public:
        real radius;
    };

    class CollisionPlane : public CollisionPrimitive{
        public:
        Vector3 normal;
        real offset;
    };

    class CollisionBox : public CollisionPrimitive{
        public:
        Vector3 halfSize;
    };
}