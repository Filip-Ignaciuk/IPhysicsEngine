#pragma once
#include "precision.hpp"
#include "core.hpp"
#include "component.hpp"
namespace IPhysicsEngine
{
    class RigidBody : public Component{
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

        real m_maxDistanceFromCentre;

        Vector3 m_forceAccumulated;

        Vector3 m_torqueAccumulated;

        bool m_isAwake;

        Matrix4 m_transformMatrix;

        Vector3 m_lastFrameAcceleration;

    public:
        RigidBody();

        RigidBody(const Vector3& _originalPosition, const Quaternion& _originalOrientation, const real& _inverseMass, const real& _linearDamping, const real& _angularDamping, const Matrix3& _inverseInertiaTensor);

        void Integrate(real _duration);

        void AddForce(const Vector3& _vector);

        void AddForceAtPoint(const Vector3& _vector, const Vector3& _point);

        void AddForceAtBodyPoint(const Vector3& _vector, const Vector3& _point);

        void ClearAccumulators();

        void CalculateDerivedData();

        Vector3 GetPointInLocalSpace(const Vector3& _point);

        Vector3 GetPointInWorldSpace(const Vector3& _point);

        real GetMass();

        bool GetIsAwake();

        real GetInverseMass();

        Matrix3 GetInverseInertiaTensorWorld() const;

        Vector3& GetPosition();

        Quaternion GetOrientation();

        Vector3 GetRotation();

        Vector3 GetVelocity();

        Matrix4 GetTransformMatrix();

        Vector3 GetLastFrameAcceleration();

        real GetMaxDistanceFromCentre();

        bool HasFiniteMass();

        void SetPosition(Vector3& _position);

        void SetOrientation(Quaternion& _quaternion);

        void SetMass(real& _mass);

        void SetIsAwake(bool _isAwake);

        void SetInverseMass(real& _inverseMass);

        void SetLinearDamping(real& _linearDamping);

        void SetAngularDamping(real& _angularDamping);

        void SetInverseInertiaTensor(const Matrix3& _inertiaTensor);

        void SetMaxDistanceFromCentre(real _distance);

        void AddVelocity(Vector3& _velocity);

        void AddRotation(Vector3& _rotation);


    private:
        static void CalculateTransformMatrix(Matrix4& _transformMatrix, const Vector3& _position, const Quaternion& _orientation);
        static void CalculateTransformInertiaTensor(Matrix3& _iitWorld, const Quaternion& _quaternion, const Matrix3& _iitBody, const Matrix4& _rotmat);
    };
}

