#ifndef IPHYSICS_IGRAVITYMODEL_HPP
#define IPHYSICS_IGRAVITYMODEL_HPP
#include "world.hpp"

enum class GravityAlgorithm {
    Naive,
    NaiveCuda,
    BarnesHut,
    BarnesHutCuda
};

class IGravityModel {
public:
    // Constructors
    explicit IGravityModel(IPhysics::real timeStep);

    // Mutators
    void SetupSimulation();
    void UpdateSimulation();

    void UpdateNumberOfParticles(int count);

    void SetSimulationPause(bool wantsPaused);

    // Queries
    [[nodiscard]] const std::vector<IPhysics::Object*>& GetParticles();
    [[nodiscard]] bool IsSimulationPaused();

private:
    const int MAXIMUM_PARTICLE_COUNT = 100000;

    bool hasForceReg = false;

    const IPhysics::real m_timeStep;

    IPhysics::World m_world{};
    int m_numberOfParticles = 0;
    GravityAlgorithm m_gravityAlgorithm = GravityAlgorithm::BarnesHut;
    std::shared_ptr<IPhysics::ForceGenerator> m_gravityForceGenerator;

    static IPhysics::Vector3 RandomGalaxyPosition();

};

#endif
//{static_cast<IPhysics::real>(1.0f)
/// static_cast<IPhysics::real>(m_frameRate)};