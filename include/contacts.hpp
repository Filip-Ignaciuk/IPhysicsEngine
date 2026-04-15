#pragma once
#include "core.hpp"
#include "components/rigidbody.hpp"

namespace IPhysicsEngine{
    class Contact{
        friend class ContactResolver;
        public:
        RigidBody* body[2];
        real friction;
        real restitution;
        Vector3 contactPoint;
        Vector3 contactNormal;
        real penetration;
        protected:
        Matrix3 contactToWorld;
        Vector3 contactVelocity;
        real desiredDeltaVelocity;
        Vector3 relativeContactPosition[2];
        public:
        void SetBodyData(RigidBody* _one, RigidBody* _two, real _friction, real _restitution);
        protected:
        void MatchAwakeState();
        void CreateContactBasis();
        Vector3 CalculateFrictionlessImpulse(Matrix3* _inverseInertiaTensor);
        void ApplyVelocityChange(Vector3 _velocityChange[2], Vector3 _rotationChange[2]);
        void ApplyPositionChange(Vector3 _linearChange[2], Vector3 _angularChange[2], real _penetration);
        void CalculateInternals(real _duration);
        Vector3 CalculateLocalVelocity(unsigned _bodyIndex, real _duration);
        void CalculateDesiredDeltaVelocity(real _duration);
        void SwapBodies();
        void CalculateContactBasis();
    };

    class ContactResolver{
        protected:
        unsigned positionIterations;
        unsigned velocityIterations;
        real velocityEpsilon;
        real positionEpsilon;
        public:
        unsigned positionIterationsUsed;
        unsigned velocityIterationsUsed;
        void ResolveContacts(Contact* _contactArray, unsigned _numberOfContacts, real _duration);
        protected:
        void PrepareContacts(Contact* _contactArray, unsigned _numberOfContacts, real _duration);
        void AdjustVelocities(Contact* _contactArray, unsigned _numberOfContacts, real _duration);
        void AdjustPositions(Contact* _contactArray, unsigned _numberOfContacts, real _duration);
    };
}