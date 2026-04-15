#pragma once
#include <vector>

#include "object.hpp"
#include "collidebroad.hpp"
#include "collidenarrow.hpp"
#include "forcegenerator.hpp"
#include "contacts.hpp"
#include "components/primitive.hpp"

namespace IPhysicsEngine
{
    class World{
        public:
        typedef std::vector<Object*> Objects;

        static const unsigned MAX_CONTACTS = 256;

        void StartFrame();
        void RunPhysics();

        void AddObject(Object* _object);
        void AddForceRegistry(Object* _object, ForceGenerator* _forceGenerator);

        void SetTimeStep(real _timestep);
        void SetPhysicsState(bool _state);

        bool GetPhysicsState();
        real GetTimeStep();

        ForceRegistry& GetParticleForceRegistry();

        Objects& GetObjects();

        protected:
        Objects m_objects;
        BoundingVolumeHierarchyNode<BoundingSphere>* m_root = nullptr;
        ContactResolver m_contactResolver;
        ForceRegistry m_registery;
        real m_timestep = (real)1.0 / (real)60.0;
        bool m_physicsState = true;
    };
}