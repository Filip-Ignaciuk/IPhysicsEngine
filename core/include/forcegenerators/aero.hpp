#ifndef IPHYSICS_AERO_HPP
#define IPHYSICS_AERO_HPP

#include "forcegenerator.hpp"

namespace IPhysics {
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
}

#endif
