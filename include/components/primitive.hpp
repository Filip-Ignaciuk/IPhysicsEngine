#pragma once
#include "components/component.hpp"
#include "components/rigidbody.hpp"

namespace IPhysicsEngine
{
    enum CollisionPrimitiveType{
        SPHERE,
        PLANE,
        BOX
    };

    class CollisionPrimitive : public Component{
        public:
        RigidBody* rigidbody;
        Matrix4 offset;
        Matrix4 transform;
        CollisionPrimitiveType type;
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