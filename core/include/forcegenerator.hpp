#ifndef IPHYSICS_FORCEGENERATOR_HPP
#define IPHYSICS_FORCEGENERATOR_HPP
#include <vector>
#include "rigidbody.hpp"
#include "object.hpp"

namespace IPhysics{
    class ForceGenerator{
    public:
        // Deconstructors
        virtual ~ForceGenerator() = default;

        // Mutators
        virtual void UpdateForce(RigidBody* _rigidBody, real _duration) = 0;
    };

    class Gravity : public ForceGenerator{
    public:
        // Constructors
        explicit Gravity(const Vector3& _gravity);

        // Mutators
        void UpdateForce(RigidBody* _rigidBody, real _duration) override;
    private:
        Vector3 m_gravity;
    };

    class RealGravity : public ForceGenerator{
    public:
        // Constructors
        explicit RealGravity(const real& _gravityConstant);

        // Mutators
        void AddObject(Object* _object);
        void RemoveObject(Object* _object);
        void UpdateForce(RigidBody* _rigidBody, real _duration) override;
    private:
        real m_gravityConstant;
        std::vector<RigidBody*> m_rigidBodies;
    };

    class Spring : public ForceGenerator{
    public:
        // Constructors
        Spring(const Vector3& _localConnectionPoint,
            RigidBody* _other,
            const Vector3&, real
            _springConstant,
            real _restLength);

        // Mutators
        void UpdateForce(RigidBody* _rigidBody, real _duration) override;
    private:
        Vector3 m_localConnectionPoint;
        Vector3 m_localOtherConnectionPoint;
        RigidBody* m_other;
        real m_springConstant;
        real m_restLength;
    };

    class Aero : public ForceGenerator{
    public:
        // Constructors
        Aero(const Matrix3& _tensor,
            const Vector3& _localPosition,
            const Vector3* _windSpeed);

        // Mutators
        void UpdateForce(RigidBody* _rigidBody, real _duration) override;
    protected:
        Matrix3 m_tensor;
        Vector3 m_localPosition;
        const Vector3* m_windSpeed;

        void UpdateForceFromTensor(RigidBody* _body, real _duration, const Matrix3& _tensor) const;
    };

    class AeroControl : public Aero{
    public:
        // Constructors
        AeroControl(const Matrix3& _base, const Matrix3& _minimumTensor, const Matrix3& _maximumTensor, const Vector3& _localPosition, const Vector3* _windSpeed);

        // Mutators
        void SetControl(real _value);
        void UpdateForce(RigidBody* _rigidBody, real _duration) override;

    protected:
        Matrix3 m_maxTensor;
        Matrix3 m_minTensor;
        real m_controlSetting;

    private:
        Matrix3 GetTensor();
        
    };

    struct ForceRegistration{
        RigidBody* rigidBody;
        std::shared_ptr<ForceGenerator> forceGenerator;

        bool operator==(const ForceRegistration& _other) const {
            return rigidBody == _other.rigidBody && forceGenerator == _other.forceGenerator;
        }
    };

    class ForceRegistry{
    public:
        // Constructors
        ForceRegistry();

        // Mutators
        void Add(Object* _object, const std::shared_ptr<ForceGenerator> &_forceGenerator);
        void Remove(Object* _object, const std::shared_ptr<ForceGenerator>& _forceGenerator);
        void Remove(Object* _object);
        void RemoveAll();
        void Clear();
        void UpdateForces(real _duration);

        // Queries
        ForceRegistration* Get(Object* _object) const;

    protected:
        std::vector<ForceRegistration> registrations;
    };
}

#endif