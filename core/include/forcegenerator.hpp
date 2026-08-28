#ifndef IPHYSICS_FORCEGENERATOR_HPP
#define IPHYSICS_FORCEGENERATOR_HPP

#include <vector>
#include <memory>

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