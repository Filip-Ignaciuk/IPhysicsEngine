#ifndef IPHYSICS_IGRAVITYMODEL_HPP
#define IPHYSICS_IGRAVITYMODEL_HPP
#include "world.hpp"
#include <string>

enum class GravityAlgorithm { Naive, NaiveCuda, BarnesHut, BarnesHutCuda };

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

  // CUDA
  [[nodiscard]] bool HasCUDA();
  [[nodiscard]] const std::string& GetDeviceName();
  [[nodiscard]] const std::string& GetComputeCapability();
  [[nodiscard]] const std::string& GetTotalGlobalMemory();
  [[nodiscard]] const std::string& GetMultiProcessorCount();
  

 private:
  const int MAXIMUM_PARTICLE_COUNT = 100000;

  bool hasForceReg = false;

  const IPhysics::real m_timeStep;

  IPhysics::World m_world{};
  int m_numberOfParticles = 0;
  GravityAlgorithm m_gravityAlgorithm = GravityAlgorithm::BarnesHut;
  std::shared_ptr<IPhysics::ForceGenerator> m_gravityForceGenerator;

  bool hasCUDA = false;
  std::string m_deviceName;
  std::string m_computeCapability;
  std::string m_totalGlobalMemory;
  std::string m_multiProcessorCount;

  static IPhysics::Vector3 RandomGalaxyPosition();
};

//{static_cast<IPhysics::real>(1.0f)
/// static_cast<IPhysics::real>(m_frameRate)};

#endif