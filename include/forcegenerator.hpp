#pragma once
#include <vector>

#include "rigidbody.hpp"

namespace IPhysicsEngine{
    class ForceGenerator{
    public:
        virtual void UpdateForce(RigidBody* _rigidBody, real _duration) = 0;
    };

    class Gravity : public ForceGenerator{
        Vector3 m_gravity;
    public:
        Gravity(const Vector3& _gravity);

        virtual void UpdateForce(RigidBody* _rigidBody, real _duration);
    };

    class Spring : public ForceGenerator{
        Vector3 m_connectionPoint;
        Vector3 m_otherConnectionPoint;
        RigidBody* m_other;
        real m_springConstant;
        real m_restLength;
    public:
        Spring(const Vector3& _localConnectionPoint, RigidBody* _other, const Vector3& _otherConnectionPoint, real _springConstant, real _restLength);
        virtual void UpdateForce(RigidBody* _rigidBody, real _duration);
    };

    struct ForceRegistration{
        RigidBody* rigidBody;
        ForceGenerator* forceGenerator;

        bool operator==(const ForceRegistration& _other) const {
            return rigidBody == _other.rigidBody && forceGenerator == _other.forceGenerator;
        }
    };

    class Aero : public ForceGenerator{
    protected:
        Matrix3 tensor;
        Vector3 position;
        const Vector3* windspeed;

        void UpdateForceFromTensor(RigidBody* _body, real _duration, const Matrix3& _tensor);
    public:
        Aero(const Matrix3& _tensor, const Vector3& _position, const Vector3* _windspeed);
        virtual void UpdateForce(RigidBody* _rigidBody, real _duration);
    };

    struct ForceRegistration{
        RigidBody* rigidBody;
        ForceGenerator* forceGenerator;

        bool operator==(const ForceRegistration& _other) const {
            return rigidBody == _other.rigidBody && forceGenerator == _other.forceGenerator;
        }
    };


    class ForceRegistry{
        protected:
        
        std::vector<ForceRegistration> registrations;
        public:
            ForceRegistry();
            void Add(RigidBody* _rigidbody, ForceGenerator* _forceGenerator);
            void Remove(RigidBody* _rigidbody, ForceGenerator* _forceGenerator);
            void Clear();
            void UpdateForces(real _duration);

    };
}