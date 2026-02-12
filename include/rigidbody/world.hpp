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
        ForceRegistry m_registery;
        IPhysicsEngine::real m_timestep = (real)1.0 / (real)60.0;
        bool m_physicsState = true;
    };
}