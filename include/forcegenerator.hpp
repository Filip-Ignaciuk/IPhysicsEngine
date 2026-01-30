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
        Vector3 m_localConnectionPoint;
        Vector3 m_localOtherConnectionPoint;
        RigidBody* m_other;
        real m_springConstant;
        real m_restLength;
    public:
        Spring(const Vector3& _localConnectionPoint, RigidBody* _other, const Vector3&, real _springConstant, real _restLength);
        virtual void UpdateForce(RigidBody* _rigidBody, real _duration);
    };

    class Aero : public ForceGenerator{
    protected:
        Matrix3 m_tensor;
        Vector3 m_localPosition;
        const Vector3* m_windspeed;

        void UpdateForceFromTensor(RigidBody* _body, real _duration, const Matrix3& _tensor);
    public:
        Aero(const Matrix3& _tensor, const Vector3& _localPosition, const Vector3* _windspeed);
        virtual void UpdateForce(RigidBody* _rigidBody, real _duration);
    };

    class AeroControl : public Aero{
    private:
        Matrix3 GetTensor();
    protected:
        Matrix3 m_maxTensor;
        Matrix3 m_minTensor;
        real m_controlSetting;
    public:
        AeroControl(const Matrix3& _base, const Matrix3& _minimumTensor, const Matrix3& _maximumTensor, const Vector3& _localPosition, const Vector3* _windspeed);
        void SetControl(real _value);
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