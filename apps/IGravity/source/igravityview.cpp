#include "igravityview.hpp"

#include <cmath>
#include <iostream>

#include "raymath.h"
#include "raygui.h"


IGravityView::IGravityView(IGravityModel* iGravityModel,
                           IGravityController* iGravityController,
                           int screenWidth, int screenHeight)
    : m_iGravityModel(iGravityModel),
      m_iGravityController(iGravityController),
      m_screenWidth(screenWidth),
      m_screenHeight(screenHeight) {
  camera = {static_cast<float>(m_screenWidth) / 2,
            static_cast<float>(m_screenHeight) / 2};
  camera.zoom = 1.0f;
}

void IGravityView::Display() {
  UpdateCamera();

  UpdateControls();

  BeginDrawing();

  ClearBackground(DARKGRAY);

  


  BeginMode2D(camera);

  UpdateParticles();

  EndMode2D();

  EndDrawing();
}

void IGravityView::UpdateCamera() {
  if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    const Vector2 delta = Vector2Scale(
        m_iGravityController->GetLatestMouseDelta(), -1.0f / camera.zoom);
    camera.target = Vector2Add(camera.target, delta);
  }

  if (m_iGravityController->GetLatestMouseWheelMove() != 0) {
    Vector2 mouseWorldPos = GetScreenToWorld2D(
        m_iGravityController->GetLatestMousePosition(), camera);
    camera.offset = m_iGravityController->GetLatestMousePosition();
    camera.target = mouseWorldPos;
    float scale = 0.2f * m_iGravityController->GetLatestMouseWheelMove();
    camera.zoom = Clamp(expf(logf(camera.zoom) + scale), 0.125f, 64.0f);
  }
}

void IGravityView::UpdateControls() {
  if (IsKeyPressed(KEY_SPACE)) {
    m_iGravityModel->SetSimulationPause(!m_iGravityModel->IsSimulationPaused());
  }
}

void IGravityView::UpdateUI(){
  UpdateErrorMessages();
  UpdateTopButtons();
  UpdateLeftHandGuiState();

}

void IGravityView::UpdateErrorMessages(){

}

void IGravityView::UpdateTopButtons(){
  // Settings Button
  if (GuiButton((Rectangle){1224, 24, 24, 24}, "#142#")) {
    if (leftHandSideGuiState == LeftHandSideGuiState::SettingsBox) {
      leftHandSideGuiState = LeftHandSideGuiState::None;
    } else {
      leftHandSideGuiState = LeftHandSideGuiState::SettingsBox;
    }
  }

  // Parameters Button
  if (GuiButton((Rectangle){24, 24, 24, 24}, "#214#")) {
    if (leftHandSideGuiState == LeftHandSideGuiState::ParametersBox) {
      leftHandSideGuiState = LeftHandSideGuiState::None;
    } else {
      leftHandSideGuiState = LeftHandSideGuiState::ParametersBox;
    }
  }

  // Pause Button
  if (GuiButton((Rectangle){168, 24, 24, 24}, pauseButtonText.c_str())) {
    if (m_iGravityModel->IsSimulationPaused()) {
      pauseButtonText = "#131#";
      m_iGravityModel->SetSimulationPause(false);
    } else {
      pauseButtonText = "#132#";
      m_iGravityModel->SetSimulationPause(true);
    }
  }

  // Help Button
  if (GuiButton((Rectangle){120, 24, 24, 24}, "#193#")) {
    if (leftHandSideGuiState == LeftHandSideGuiState::HelpBox) {
      leftHandSideGuiState = LeftHandSideGuiState::None;
    } else {
      leftHandSideGuiState = LeftHandSideGuiState::HelpBox;
    }
  }
}

void IGravityView::UpdateLeftHandGuiState(){
  if(leftHandSideGuiState == LeftHandSideGuiState::SettingsBox){
    DisplaySettingsMenu();
  }
  else if(leftHandSideGuiState == LeftHandSideGuiState::ParametersBox){
    
  }
  else if(leftHandSideGuiState == LeftHandSideGuiState::HelpBox){

  }
  else{
    return;
  }
}

void IGravityView::DisplaySettingsMenu(){
  if (GuiWindowBox(standardLeftBox, "Settings")) {
    leftHandSideGuiState = LeftHandSideGuiState::None;
  } 
  else {
    leftHandSideGuiState = LeftHandSideGuiState::SettingsBox;

    // CUDA
    GuiLabel(
      (Rectangle){standardLeftBox.x + 24, standardLeftBox.y + 24, 96, 24},
      "CUDA Compatiblity");

    if(IApp::GPUInformation::HasCUDA()){
      std::cout << "Device Name: " << IApp::GPUInformation::GetDeviceName() << std::endl;
      std::cout << "Compute Capability: " << IApp::GPUInformation::GetComputeCapability() << std::endl;
      std::cout << "Total Global Memory: " << IApp::GPUInformation::GetTotalGlobalMemory() << std::endl;
      std::cout << "Multi Processor Count: " << IApp::GPUInformation::GetMultiProcessorCount() << std::endl;
    }
    else{

    }

  }
}

void IGravityView::UpdateParticles() const {
  for (int i = 0; i < m_iGravityModel->GetParticles().size(); ++i) {
    IPhysics::Object* obj = m_iGravityModel->GetParticles()[i];
    const auto* rigidBody = obj->GetComponent<IPhysics::RigidBody>();
    const Vector2 position{.x = static_cast<float>(rigidBody->GetPosition().x),
                           .y = static_cast<float>(rigidBody->GetPosition().y)};
    DrawCircleV(position, 1.0f, RED);
  }
}
