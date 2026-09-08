#ifndef IPHYSICS_AERO_HPP
#define IPHYSICS_AERO_HPP

#include "forcegenerator.hpp"

namespace IPhysics {
    class Aero : public ForceGenerator{
    public:
        // Constructors
        Aero(const Matrix3& tensor,
            const Vector3& localPosition,
            const Vector3* windSpeed);

        // Mutators
        void UpdateForce(RigidBody* rigidBody, real duration) override;
    protected:
        Matrix3 m_tensor;
        Vector3 m_localPosition;
        const Vector3* m_windSpeed;

        void UpdateForceFromTensor(RigidBody* body, real duration, const Matrix3& tensor) const;
    };

    class AeroControl : public Aero{
    public:
        // Constructors
        AeroControl(const Matrix3& base, const Matrix3& minimumTensor, const Matrix3& maximumTensor, const Vector3& localPosition, const Vector3* windSpeed);

        // Mutators
        void SetControl(real value);
        void UpdateForce(RigidBody* rigidBody, real duration) override;

    protected:
        Matrix3 m_maxTensor;
        Matrix3 m_minTensor;
        real m_controlSetting;

    private:
        Matrix3 GetTensor();

    };
}

#endif
