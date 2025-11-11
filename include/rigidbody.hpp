#pragma once
#include "precision.hpp"
#include "core.hpp"

namespace IPhysicsEngine
{
    class RigidBody{
    protected:
        real m_inverseMass;
        real m_linearDamping;
        Vector3 m_position;
        Quaternion m_orientation;
        Vector3 m_velocity;
        Vector3 m_acceleration;
        Vector3 m_rotation;
        Matrix3 m_inverseInertiaTensor;
        Matrix3 m_inverseInertiaTensorWorld;
        real m_angularDamping;


        Vector3 m_forceAccumulated;

        Vector3 m_torqueAccumulated;

        bool m_isAwake;

        Matrix4 m_transformMatrix;

        Vector3 m_lastFrameAcceleration;

    public:
        RigidBody(const Vector3& _originalPosition, const Quaternion& _originalOrientation, const real& _inverseMass, const real& _linearDamping, const real& _angularDamping, const Matrix3& _inverseInertiaTensor);

        void Integrate(real _duration);

        void AddForce(const Vector3& _vector);

        void AddForceAtPoint(const Vector3& _vector, const Vector3& _point);

        void AddForceAtBodyPoint(const Vector3& _vector, const Vector3& _point);

        void SetInertiaTensor(const Matrix3& _inertiaTensor);

        void ClearAccumulators();

        void CalculateDerivedData();

        Vector3 GetPointInLocalSpace(const Vector3& _point);

        Vector3 GetPointInWorldSpace(const Vector3& _point);

        real GetMass();

        Vector3& GetPosition();

        bool HasFiniteMass();

    private:
        static void CalculateTransformMatrix(Matrix4& _transformMatrix, const Vector3& _position, const Quaternion& _orientation);
        static void CalculateTransformInertiaTensor(Matrix3& _iitWorld, const Quaternion& _quaternion, const Matrix3& _iitBody, const Matrix4& _rotmat);
    };
}

