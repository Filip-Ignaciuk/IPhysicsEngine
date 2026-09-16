#ifndef IGRAVITY_IGRAVITYMODEL_HPP
#define IGRAVITY_IGRAVITYMODEL_HPP

#include "core.hpp"
#include "gravity.hpp"
#include "world.hpp"

#include <map>
#include <string>

enum class GravityAlgorithm { Naive, BarnesHut, NaiveCuda };

static const std::vector<std::string> algorithmText ={
  "Naive Algorithm",
  "Barnes Hut Algorithm",
  "Cuda Algorithm"
};

static std::map<std::string, GravityAlgorithm> GravityAlgorithmTextMap = {
  {algorithmText[0], GravityAlgorithm::Naive},
  {algorithmText[1], GravityAlgorithm::BarnesHut},
  {algorithmText[2], GravityAlgorithm::NaiveCuda}
};

class IGravityModel {
 public:
  // Constructors
  explicit IGravityModel(IPhysics::real timeStep);

  // Mutators
  void SetupSimulation();
  void UpdateSimulation();

  void UpdateNumberOfParticles(int count);
  void UpdateAlgorithmType(GravityAlgorithm gravityAlgorithm);

  void SetSimulationPause(bool wantsPaused);

  void Restart();

  // Queries
  [[nodiscard]] const std::vector<IPhysics::Object*>& GetParticles();
  [[nodiscard]] bool IsSimulationPaused() const;
  [[nodiscard]] const int GetMaximumParticleCount() const;
  

 private:
  static const int MAXIMUM_PARTICLE_COUNT = 100000;

  const IPhysics::real GRAVITY_CONSTANT = 6.674 * pow(10, -11);

  bool hasForceReg = false;

  const IPhysics::real m_timeStep;

  IPhysics::World m_world{};
  GravityAlgorithm m_gravityAlgorithm = GravityAlgorithm::Naive;
  std::shared_ptr<IPhysics::Gravity> m_gravityForceGenerator;

  static IPhysics::Vector3 RandomGalaxyPosition();

  void CalculateParticleVelocities();

  IPhysics::Vector3 GetGalaxyVelocity();

  bool IsUsingCUDAAlgorithm() const;
};

//{static_cast<IPhysics::real>(1.0f)
/// static_cast<IPhysics::real>(m_frameRate)};

#endif