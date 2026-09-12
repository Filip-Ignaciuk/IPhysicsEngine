#include "igravity.hpp"

#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

IGravity::IGravity() {
  const IPhysics::real timeStep = 1.0f / m_frameRate;
  m_iGravityModel = new IGravityModel(timeStep);
  m_iGravityController = new IGravityController();
  m_iGravityView = new IGravityView(m_iGravityModel, m_iGravityController,
                                    m_screenWidth, m_screenHeight);

}

void IGravity::Run() const {
  InitWindow(m_screenWidth, m_screenHeight, "IGravity");
  TraceLog(LOG_INFO, "window opened");
  SetTargetFPS(m_frameRate);

  while (!WindowShouldClose()) {
    m_iGravityController->Update();
    m_iGravityModel->UpdateSimulation();
    m_iGravityView->Display();
  }

  CloseWindow();
}
