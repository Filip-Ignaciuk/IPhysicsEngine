#pragma once
#include "core.hpp"

namespace IPhysicsEngine{
    class Contact{
        public:
        RigidBody* body[2];
        real friction;
        real restitution;
        Vector3 contactPoint;
        Vector3 contactNormal;
        real penetration;
        protected:
        Matrix3 contactToWorld;
        real desiredDeltaVelocity;
        Vector3 relativeContactPosition[2];
        public:
        void SetBodyData(RigidBody* _one, RigidBody* _two, real _friction, real _restitution);
        void CreateContactBasis();
        Vector3 CalculateFrictionlessImpulse(Matrix3* _inverseInertiaTensor);
    };
}