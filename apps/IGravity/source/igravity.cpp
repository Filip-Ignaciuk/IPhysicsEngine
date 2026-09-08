#include "igravity.hpp"

#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

IGravity::IGravity() {
  constexpr IPhysics::real timeStep = 1.0f / 60.0f;
  m_iGravityModel = new IGravityModel(timeStep);
  m_iGravityController = new IGravityController();
  m_iGravityView = new IGravityView(m_iGravityModel, m_iGravityController,
                                    m_screenWidth, m_screenHeight);

  m_iGravityModel->SetupSimulation();
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
