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

        RigidBody* rigidbody;

        bool isLead() const;
        unsigned GetPotentialContacts(PotentialContact* _contacts, unsigned _limit) const;
        
    };
    
}