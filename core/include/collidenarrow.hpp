#pragma once
#include "contacts.hpp"
#include "components/primitive.hpp"

namespace IPhysics{
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
        public:
        static real TransformToAxis(const CollisionBox& _box, const Vector3& _axis);
        static bool OverlapOnAxis(const CollisionBox& _box1, const CollisionBox& _box2, const Vector3& _axis, const Vector3& _toCentre);
        static bool BoxAndHalfSpace(const CollisionBox& _box, const CollisionPlane& _plane);
        static bool BoxAndBox(const CollisionBox& _box1, const CollisionBox& _box2);
    };

    class CollisionDetector{
        public:
        static unsigned SphereAndSphere(const CollisionSphere& _firstSphere, const CollisionSphere& _secondSphere, CollisionData* _data);
        static unsigned SphereAndHalfSpace(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned SphereAndPlane(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned BoxAndHalfSpace(const CollisionBox& _box, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned BoxAndPlane(const CollisionBox& _box, const CollisionPlane& _plane, CollisionData* _data);
        static unsigned BoxAndSphere(const CollisionBox& _box, const CollisionSphere& _sphere, CollisionData* _data);
        static unsigned BoxAndBox(const CollisionBox& _box1, const CollisionBox& _box2, CollisionData* _data);

        private:
        static bool TryAxis(const CollisionBox& _box1, const CollisionBox& _box2, Vector3 _axis, const Vector3& _toCentre, unsigned _index, real& _smallestPenetration, unsigned& _smallestCase);
        static real PenetrationOnAxis(const CollisionBox& _box1, const CollisionBox& _box2, const Vector3& _axis, const Vector3& _toCentre);
        static void FillPointFaceBoxBox(const CollisionBox& _box1, const CollisionBox& _box2, const Vector3& _toCentre, CollisionData* _data, unsigned _best, real _penetration);
        static Vector3 ContactPoint(const Vector3& _pointOnOneEdge, const Vector3& _oneAxis, real _oneSize, const Vector3& _pointOnTwoEdge, const Vector3& _twoAxis, real _twoSize, bool _useOne);
    };
}