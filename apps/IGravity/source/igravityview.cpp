#include "igravityview.hpp"

#include <cmath>
#include <iostream>
#include <raylib.h>

#include "igravitymodel.hpp"
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

IGravityView::~IGravityView(){

}

void IGravityView::Display() {
  BeginDrawing();

  ClearBackground(DARKGRAY);

  BeginMode2D(camera);

  UpdateParticles();

  EndMode2D();

  UpdateUI();

  UpdateCamera();

  UpdateControls();

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
    UpdateSimulationState(!m_iGravityModel->IsSimulationPaused());
  }
}

void IGravityView::UpdateUI(){
  UpdateErrorMessages();
  UpdateTopButtons();
  UpdateLeftHandGuiState();

}

void IGravityView::UpdateSimulationState(bool wantsPaused){
  if (!wantsPaused && m_iGravityModel->IsSimulationPaused()) {
      pauseButtonText = "#131#";
      m_iGravityModel->SetSimulationPause(false);
    } else {
      pauseButtonText = "#132#";
      m_iGravityModel->SetSimulationPause(true);
    }
}

void IGravityView::UpdateErrorMessages(){

}

void IGravityView::UpdateTopButtons(){
  // Settings Button
  if (GuiButton((Rectangle){24, 24, 24, 24}, "#142#")) {
    if (leftHandSideGuiState == LeftHandSideGuiState::SettingsBox) {
      leftHandSideGuiState = LeftHandSideGuiState::None;
    } else {
      leftHandSideGuiState = LeftHandSideGuiState::SettingsBox;
    }
  }

  // Parameters Button
  if (GuiButton((Rectangle){72, 24, 24, 24}, "#214#")) {
    if (leftHandSideGuiState == LeftHandSideGuiState::ParametersBox) {
      leftHandSideGuiState = LeftHandSideGuiState::None;
    } else {
      leftHandSideGuiState = LeftHandSideGuiState::ParametersBox;
    }
  }

  // Restart Button
  if (GuiButton((Rectangle){120, 24, 24, 24}, "#58#")) {
    m_iGravityModel->Restart();
  }

  // Help Button
  if (GuiButton((Rectangle){164, 24, 24, 24}, "#193#")) {
    if (leftHandSideGuiState == LeftHandSideGuiState::HelpBox) {
      leftHandSideGuiState = LeftHandSideGuiState::None;
    } else {
      leftHandSideGuiState = LeftHandSideGuiState::HelpBox;
    }
  }

  // Pause Button
  if (GuiButton((Rectangle){212, 24, 24, 24}, pauseButtonText.c_str())) {
    if (m_iGravityModel->IsSimulationPaused()) {
      UpdateSimulationState(false);
    } else {
      UpdateSimulationState(true);
    }
  }

  
}

void IGravityView::UpdateLeftHandGuiState(){
  if(leftHandSideGuiState == LeftHandSideGuiState::SettingsBox){
    DisplaySettingsMenu();
  }
  else if(leftHandSideGuiState == LeftHandSideGuiState::ParametersBox){
    DisplayParametersMenu();
  }
  else if(leftHandSideGuiState == LeftHandSideGuiState::HelpBox){
    DisplayHelpMenu();
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
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 24, 
        96, 
        24},
      "CUDA Compatiblity");

    std::string hasCudaText;

    Color compatibilityColor = BLACK;

    if(IApp::GPUInformation::HasCUDA()){
      hasCudaText = "CUDA Capable Device Detected!";
      compatibilityColor = GREEN;
    }
    else{
      hasCudaText = "Unable to detect CUDA Capable Device.";
      compatibilityColor = RED;
    }

    // Change colour based on whether they have a CUDA capable card or not.
    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, ColorToInt(GREEN));

    GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 48, 
        256, 
        24},
      hasCudaText.c_str());

    if(IApp::GPUInformation::HasCUDA()){

      std::string deviceNameText = "Device Name: " + 
      IApp::GPUInformation::GetDeviceName();

      GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 72, 
        256, 
        24},
      deviceNameText.c_str());

      std::string computeText = "Compute Capability: " + 
      IApp::GPUInformation::GetComputeCapability();

      GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 96, 
        256, 
        24},
      computeText.c_str());

      std::string memoryText = "Total Global Memory: " + 
      IApp::GPUInformation::GetTotalGlobalMemory();

      GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 120, 
        256, 
        24},
      memoryText.c_str());

      std::string ProcessorText = "Multi Processor Count: " + 
      IApp::GPUInformation::GetMultiProcessorCount();

      GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 144, 
        256, 
        24},
      ProcessorText.c_str());
    }

    // Change colour based on whether they have a CUDA capable card or not.
    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, ColorToInt(BLACK));
  }
}

void IGravityView::DisplayParametersMenu(){
  if (GuiWindowBox(standardLeftBox, "Simulation Parameters")) {
    leftHandSideGuiState = LeftHandSideGuiState::None;
  } 
  else {
    leftHandSideGuiState = LeftHandSideGuiState::ParametersBox;

    if(dropdownIsEditMode){
      GuiLock();
    }

    std::string numberOfParticlesText = "Number of particles: " +
      std::to_string(m_iGravityModel->GetParticles().size());

    GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 24, 
        256, 
        24},
      numberOfParticlesText.c_str());

    GuiSlider((Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 48, 
        256, 
        24}, 
        "" ,
        "", 
        &parametersSliderValue, 
        0, 
        m_iGravityModel->GetMaximumParticleCount());

    numberOfParticlesDesired = roundf(parametersSliderValue);
    if(numberOfParticlesDesired != m_iGravityModel->GetParticles().size()){
      int newCount = numberOfParticlesDesired - m_iGravityModel->GetParticles().size();
      m_iGravityModel->UpdateNumberOfParticles(newCount);
    }

    std::string currentAlgorithmText = "Algorithm Type: " +
      algorithmText[activeAlgorithmDropdownValue];


    std::string dropdownAlgorithmText = 
    algorithmText[0] + ";" +
    algorithmText[1] + ";" +
    algorithmText[2] + ";" +
    algorithmText[3];

    GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 72, 
        256, 
        24},
      currentAlgorithmText.c_str());

    int previousAlgorithmDropdownValue = activeAlgorithmDropdownValue;

    if(GuiDropdownBox((Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 96, 
        256, 
        24}, dropdownAlgorithmText.c_str(), 
    &activeAlgorithmDropdownValue, 
    dropdownIsEditMode)){
      dropdownIsEditMode = !dropdownIsEditMode;
     
    }

    if(previousAlgorithmDropdownValue != activeAlgorithmDropdownValue){
      std::cout <<algorithmText[activeAlgorithmDropdownValue] << std::endl;
       m_iGravityModel->UpdateAlgorithmType(
        GravityAlgorithmTextMap[algorithmText[activeAlgorithmDropdownValue]]);
    }

    GuiUnlock();

  }
}

void IGravityView::DisplayHelpMenu(){
  if (GuiWindowBox(standardLeftBox, "Simulation Parameters")) {
    leftHandSideGuiState = LeftHandSideGuiState::None;
  } 
  else {
    leftHandSideGuiState = LeftHandSideGuiState::HelpBox;
    GuiLabel(
      (Rectangle){
        standardLeftBox.x + 24, 
        standardLeftBox.y + 24, 
        256, 
        24},
      "Welcome to IGravity!");
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
