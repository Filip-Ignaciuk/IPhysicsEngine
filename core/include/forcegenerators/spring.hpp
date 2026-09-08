#ifndef IPHYSICS_SPRING_HPP
#define IPHYSICS_SPRING_HPP

#include "forcegenerator.hpp"

#include "core.hpp"

namespace IPhysics {
    class Spring : public ForceGenerator {
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
        IPhysics::RigidBody* m_other;
        IPhysics::real m_springConstant;
        real m_restLength;
    };
}
#endif
