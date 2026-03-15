#pragma once
#include "rigidbody.hpp"

namespace IPhysicsEngine
{
    class CollisionPrimitive{
        public:
        RigidBody* rigidbody;
        Matrix4 offset;

        Vector3 GetAxis(unsigned _index) const;
        protected:
        Matrix4 transform;
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