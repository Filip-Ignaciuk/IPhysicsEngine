#ifndef IGRAVITY_IGRAVITYVIEW_HPP
#define IGRAVITY_IGRAVITYVIEW_HPP

#include <raylib.h>

#include "igravitycontroller.hpp"
#include "igravitymodel.hpp"

enum class LeftHandSideGuiState { None, SettingsBox, ParametersBox, HelpBox };

class IGravityView {
 public:
  // Constructors
  IGravityView(IGravityModel* iGravityModel,
               IGravityController* iGravityController, int screenWidth,
               int screenHeight);

  ~IGravityView();

  // Mutators
  void Display();

 private:
  IGravityModel* m_iGravityModel;
  IGravityController* m_iGravityController;

  const int m_screenWidth;
  const int m_screenHeight;

  Camera2D camera;

  LeftHandSideGuiState leftHandSideGuiState = LeftHandSideGuiState::None;

  constexpr static Rectangle standardLeftBox = 
    {24, 72, 408, 408};

  std::string pauseButtonText = "#132#";

  int activeAlgorithmDropdownValue = 0;
  bool dropdownIsEditMode = false;

  int numberOfParticlesDesired = 0;
  GravityAlgorithm gravityAlgorithmDesired = GravityAlgorithm::Naive;
  IPhysics::real barnesHutAccuracyDesired = 0.5;

  void UpdateCamera();

  void UpdateControls();

  void UpdateUI();

  void UpdateSimulationState(bool wantsPause);

  // Related to UpdateUI
  void UpdateErrorMessages();
  void UpdateTopButtons();
  void UpdateLeftHandGuiState();

  void DisplaySettingsMenu();
  void DisplayParametersMenu();
  void DisplayHelpMenu();

  float parametersSliderValue = 1000;

  void UpdateParticles() const;
};


#endif
