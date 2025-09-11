#pragma once
#include "core.hpp"

namespace IPhysicsEngine{
    class Particle{
        protected:
            Vector3 m_position;
            Vector3 m_velocity;
            Vector3 m_acceleration;
            Vector3 m_forceAccumulated;
            real m_damping;
            real m_inverseMass;

        public:
            Particle();
            Particle(Vector3 _position, real _damping, real _inverseMass);

            void SetMass(real _mass);
            void SetInverseMass(real _inverseMass);
            void SetDamping(real _damping);
            void SetPosition(Vector3 _position);
            void SetVelocity(Vector3 _velocity);
            void SetAcceleration(Vector3 _acceleration);
            void AddForce(Vector3 _force);
            virtual bool Integrate(real _duration);
            void ClearAccumulator();
        
            real GetKineticEnergy() const;
            Vector3 GetPosition() const;
            Vector3 GetVelocity() const;
            Vector3 GetAcceleration() const;

            

    };
}

