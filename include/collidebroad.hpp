#pragma once
#include "rigidbody.hpp"

namespace IPhysicsEngine{
    struct PotentialContact
    {
        RigidBody* body[2];
    };

    template<class BoundingVolumeClass>
    class BoundingVolumeHierarchyNode{
        public:
        BoundingVolumeHierarchyNode* children[2];

        BoundingVolumeClass volume;

        RigidBody* body;

        bool IsLeaf() const;
        unsigned GetPotentialContacts(PotentialContact* _contacts, unsigned _limit) const;
        bool Overlaps(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other) const;
        unsigned GetPotentialContactsWith(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other, PotentialContact* _contacts, unsigned _limit) const;


    };
    
}

namespace IPhysicsEngine{
    class BoundingSphere{
        private:
        Vector3 m_centre;
        real m_radius;
        public:
        BoundingSphere(const Vector3& _centre, real _radius);
        BoundingSphere(const BoundingSphere& _one, const BoundingSphere& _two);
        bool Overlaps(const BoundingSphere* _other) const;
    };
}