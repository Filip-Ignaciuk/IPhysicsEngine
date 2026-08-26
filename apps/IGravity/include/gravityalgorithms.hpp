#include "forcegenerator.hpp"

class RealGravityCuda : public IPhysics::ForceGenerator {
private:
    IPhysics::real m_gravityConstant;
    std::vector<IPhysics::RigidBody*> m_rigidBodies;
public:
    RealGravityCuda(IPhysics::real _gravityConstant);
    void AddObject(IPhysics::Object* _object);
    void RemoveObject(IPhysics::Object* _object);
    void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;
};

struct BHTN {

    ~BHTN() {
        delete nw;
        delete ne;
        delete sw;
        delete se;
    }

    // Data
    IPhysics::real width = 0;
    IPhysics::Vector3 midPoint;

    IPhysics::RigidBody* rigidBody = nullptr;
    IPhysics::real mass = 0;
    IPhysics::Vector3 centreOfMass{};

    BHTN* nw = nullptr;
    BHTN* ne = nullptr;
    BHTN* sw = nullptr;
    BHTN* se = nullptr;

    [[nodiscard]] bool IsExternalNode() const{
        return nw == nullptr
        && ne  == nullptr
        && sw == nullptr
        && se  == nullptr;
    }
};

class RealGravityBarnesHut : public IPhysics::ForceGenerator {
public:
    // Constructors
    RealGravityBarnesHut(IPhysics::real _gravityConstant,
        IPhysics::real thresholdValue);

    // Mutators
    void AddObject(IPhysics::Object* _object);
    void RemoveObject(IPhysics::Object* _object);

    void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;

private:
    IPhysics::real m_gravityConstant;
    std::vector<IPhysics::RigidBody*> m_rigidBodies;
    int totalProcessedParticles = 0;

    BHTN* root;
    IPhysics::real thresholdValue;
    IPhysics::real minWidth = 0.001;

    void CreateTree();
    void CreateTreeRoot();

    void AddObjectToNode(BHTN* _node, IPhysics::RigidBody* _rigidBody);

    IPhysics::Vector3 CalculateGravityForce(
        IPhysics::real _mass1,
        const IPhysics::Vector3& _centreOfMass1,
        IPhysics::real _mass2,
        const IPhysics::Vector3& _centreOfMass2) const;

    IPhysics::Vector3 TraverseNode(
        const BHTN* _node,
        const IPhysics::RigidBody* _rigidBody);
};

class RealGravityBarnesHutCuda : public IPhysics::ForceGenerator {
private:
    IPhysics::real m_gravityConstant;
    std::vector<IPhysics::RigidBody*> m_rigidBodies;
    void CreateTree();
public:
    RealGravityBarnesHutCuda(const IPhysics::real& _gravityConstant);

    void AddObject(IPhysics::Object* _object);
    void RemoveObject(IPhysics::Object* _object);

    void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;
};