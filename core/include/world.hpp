#pragma once
#include <vector>
#include <algorithm>

#include "object.hpp"
#include "collidebroad.hpp"
#include "collidenarrow.hpp"
#include "forcegenerator.hpp"
#include "contacts.hpp"
#include "components/primitive.hpp"

namespace IPhysics
{
    class World{
        public:
        typedef std::vector<Object*> Objects;

        static const unsigned MAX_CONTACTS = 256;

        void StartFrame();
        void RunPhysics(real _timestep);

        void AddObject(Object* _object);
        void AddForceRegistry(Object* _object, ForceGenerator* _forceGenerator);

        void SetPhysicsState(bool _state);

        bool GetPhysicsState();

        ForceRegistry& GetParticleForceRegistry();

        Objects& GetObjects();

        void RemoveObject(Object* _object);
        void RemoveForceRegistry(Object* _object);

        protected:
        Objects m_objects;
        BoundingVolumeHierarchyNode<BoundingSphere>* m_root = nullptr;
        ContactResolver m_contactResolver;
        ForceRegistry m_registery;
        bool m_physicsState = true;
    };
}