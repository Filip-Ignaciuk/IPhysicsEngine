#pragma once
#include "contacts.hpp"
#include "primitive.hpp"

namespace IPhysicsEngine{
    struct CollisionData
    {
        Contact* contactsArray;
        Contact* contacts;
        int contactsLeft;
        unsigned contactCount;
        real friction;
        real restitution;
        real tolerance;

        void AddContacts(unsigned _count);
    };

    class CollisionDetector{
        public:
            static unsigned SphereAndSphere(const CollisionSphere& _firstPrimitive, const CollisionSphere& _secondPrimitive, CollisionData* _data);
            static unsigned SphereAndHalfSpace(const CollisionSphere& _firstPrimitive, const CollisionPlane& _secondPrimitive, CollisionData* _data);
            static unsigned SphereAndPlane(const CollisionSphere& _firstPrimitive, const CollisionPlane& _secondPrimitive, CollisionData* _data);
    };
    
}