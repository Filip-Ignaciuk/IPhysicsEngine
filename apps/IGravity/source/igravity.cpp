#include "igravity.hpp"

#include <chrono>
#include <iostream>

#include "igravitymodel.hpp"
#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "gpuinformation.hpp"


IGravity::IGravity() {
  m_timeStep = 1.0 / m_frameRate;
  m_iGravityModel = new IGravityModel{m_timeStep};
  m_iGravityController = new IGravityController{};
  m_iGravityView = new IGravityView{m_iGravityModel, m_iGravityController,
                                    m_screenWidth, m_screenHeight};

}

void IGravity::Run() {
  InitWindow(m_screenWidth, m_screenHeight, "IGravity");
  TraceLog(LOG_INFO, "window opened");
  SetTargetFPS(m_frameRate);

  IApp::GPUInformation::Initialise();

  // Benchmark to limit amount of particles based on system performance.
  Benchmark();
  
  // Update to basic algorithm.
  m_iGravityModel->UpdateAlgorithmType(GravityAlgorithm::Naive);

  // Start with an inital amount of particles
  m_iGravityModel->UpdateNumberOfParticles(IGravityModel::GetMaximumNaiveParticleCount() / 2);

  // Update initial value for IGravityView
  m_iGravityView->InitialiseParticleCount(IGravityModel::GetMaximumNaiveParticleCount() / 2);

  while (!WindowShouldClose()) {
    m_iGravityController->Update();
    m_iGravityModel->UpdateSimulation();
    m_iGravityView->Display();
  }

  CloseWindow();
}

void IGravity::Benchmark(){
  std::chrono::nanoseconds maxDuration{250000000};

  IGravityModel naiveGravityModel{static_cast<IPhysics::real>(m_timeStep)};

  naiveGravityModel.UpdateAlgorithmType(GravityAlgorithm::Naive);

  std::chrono::nanoseconds naiveDuration{0};
  int naiveParticleCount = 0;
  while(naiveDuration <= maxDuration){
    naiveGravityModel.UpdateNumberOfParticles(1000);
    auto start = std::chrono::steady_clock::now();
    naiveGravityModel.UpdateSimulation();
    auto end = std::chrono::steady_clock::now();
    naiveDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    naiveParticleCount = naiveParticleCount + 1000;
  }

  IGravityModel barnesHutGravityModel{static_cast<IPhysics::real>(m_timeStep)};

  barnesHutGravityModel.UpdateAlgorithmType(GravityAlgorithm::BarnesHut);

  std::chrono::nanoseconds barnesHutDuration{0};
  int barnesHutParticleCount = 0;
  while(barnesHutDuration <= maxDuration){
    barnesHutGravityModel.UpdateNumberOfParticles(20000);
    auto start = std::chrono::steady_clock::now();
    barnesHutGravityModel.UpdateSimulation();
    auto end = std::chrono::steady_clock::now();
    barnesHutDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    barnesHutParticleCount = barnesHutParticleCount + 20000;
  }

  // Update limits in model
  IGravityModel::SetMaximumNaiveParticleCount(naiveParticleCount);
  IGravityModel::SetMaximumBarnesHutParticleCount(barnesHutParticleCount);
}