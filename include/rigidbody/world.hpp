#pragma once
#include <vector>

#include "object.hpp"
#include "rigidbody.hpp"
#include "forcegenerator.hpp"

namespace IPhysicsEngine
{
    class World{
        public:
        typedef std::vector<Object*> Objects;

        void StartFrame();
        void RunPhysics(real _duration);

        void AddObject(Object* _object);
        void AddForceRegistry(Object* _object, ForceGenerator* _forceGenerator);

        ForceRegistry& GetParticleForceRegistry();

        Objects& GetObjects();

        protected:
        Objects m_objects;
        ForceRegistry m_registery;
    };
}