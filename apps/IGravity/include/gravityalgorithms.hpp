#include "forcegenerator.hpp"

class RealGravityCuda : public IPhysics::ForceGenerator {
private:
    IPhysics::real m_gravityConstant;
    std::vector<IPhysics::RigidBody*> m_rigidBodies;
public:
    RealGravityCuda(const IPhysics::real& _gravityConstant);
    void AddObject(IPhysics::Object* _object);
    void RemoveObject(IPhysics::Object* _object);
    void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;
};

class RealGravityBarnesHut : public IPhysics::ForceGenerator {
    private:
    void CreateTree();
public:
    RealGravityBarnesHut(const IPhysics::real& _gravityConstant);

    void AddObject(IPhysics::Object* _object);
    void RemoveObject(IPhysics::Object* _object);

    void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;
};

class RealGravityBarnesHutCuda : public IPhysics::ForceGenerator {
private:
    void CreateTree();
public:
    RealGravityBarnesHutCuda(const IPhysics::real& _gravityConstant);

    void AddObject(IPhysics::Object* _object);
    void RemoveObject(IPhysics::Object* _object);

    void UpdateForce(IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) override;
};