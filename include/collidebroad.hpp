#pragma once
#include "rigidbody/rigidbody.hpp"

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

        BoundingVolumeHierarchyNode* parent;

        BoundingVolumeHierarchyNode(BoundingVolumeHierarchyNode* _parent, const BoundingVolumeClass& _volume, RigidBody* _body = nullptr);

        bool IsLeaf() const;
        unsigned GetPotentialContacts(PotentialContact* _contacts, unsigned _limit) const;
        void Insert(RigidBody* _newBody, const BoundingVolumeClass& _newVolume);
        
        ~BoundingVolumeHierarchyNode();

        protected:
        bool Overlaps(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other) const;
        unsigned GetPotentialContactsWith(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other, PotentialContact* _contacts, unsigned _limit) const;
        void RecalculateBoundingVolume(bool recurse = true);


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
        real GetGrowth(const BoundingSphere &other) const;
    };
}