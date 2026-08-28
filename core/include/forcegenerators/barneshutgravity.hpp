#ifndef IPHYSICS_BARNESHUTGRAVITY_HPP
#define IPHYSICS_BARNESHUTGRAVITY_HPP

#include "gravity.hpp"
#include "bhtn.hpp"

namespace IPhysics {
    class BarnesHutGravity : public Gravity {
    public:
        // Constructors
        BarnesHutGravity(IPhysics::real _gravityConstant,
            IPhysics::real thresholdValue);

        // Mutators
        void AddObject(IPhysics::Object* _object) override;
        void RemoveObject(IPhysics::Object* _object) override;

        void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;

    private:
        int totalProcessedParticles = 0;

        bhtn* root = nullptr;
        IPhysics::real thresholdValue;
        IPhysics::real minWidth = 0.001;


        void CreateTree();
        void CreateTreeRoot();

        static void AddObjectToNode(bhtn* _node, IPhysics::RigidBody* _rigidBody);

        [[nodiscard]] IPhysics::Vector3 CalculateGravityForce(
            IPhysics::real _mass1,
            const IPhysics::Vector3& _centreOfMass1,
            IPhysics::real _mass2,
            const IPhysics::Vector3& _centreOfMass2) const;

        IPhysics::Vector3 TraverseNode(
            const bhtn* _node,
            const IPhysics::RigidBody* _rigidBody);
    };
}

#endif
