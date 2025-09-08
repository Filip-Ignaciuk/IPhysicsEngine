#pragma once
#include "core.hpp"

namespace IPhysicsEngine{
    class Particle{
        protected:
            Vector3 m_position;
            Vector3 m_velocity;
            Vector3 m_acceleration;
            real m_damping;
            real m_inverseMass;

        public:
            Particle();
            Particle(Vector3 _position, real _damping, real _inverseMass);

            void SetMass(real _mass);
            void SetInverseMass(real _inverseMass);
            void SetAcceleration(Vector3 _acceleration);
            void Integrate(real _duration);

            real GetKineticEnergy();
            Vector3 GetPosition();
            Vector3 GetVelocity();
            Vector3 GetAcceleration();

    };
}

