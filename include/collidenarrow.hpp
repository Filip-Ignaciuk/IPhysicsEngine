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

    // Quick tests to allow the CollisionDetector class to exit early in case they are not colliding.
    class IntersectionTests{
        private:
        static real TransformToAxis(const CollisionBox &box, const Vector3 &axis);
        public:
        static bool BoxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane);
    };

    class CollisionDetector{
        public:
        static unsigned SphereAndSphere(const CollisionSphere& _firstSphere, const CollisionSphere& _secondSphere, CollisionData* _data);
        static unsigned SphereAndHalfSpace(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned SphereAndPlane(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned BoxAndHalfSpace(const CollisionBox& _box, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned BoxAndPlane(const CollisionBox& _box, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned BoxAndSphere(const CollisionBox& _box, const CollisionSphere& _sphere, CollisionData* _data);
    };
    
}