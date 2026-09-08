#ifndef IPHYSICS_IGRAVITYVIEW_HPP
#define IPHYSICS_IGRAVITYVIEW_HPP

#include <raylib.h>

#include "igravitycontroller.hpp"
#include "igravitymodel.hpp"

class IGravityView {
public:
    // Constructors
    IGravityView(IGravityModel* iGravityModel,
        IGravityController* iGravityController,
        int screenWidth,
        int screenHeight);

    // Mutators
    void Display();


private:
    IGravityModel* m_iGravityModel;
    IGravityController* m_iGravityController;

    const int m_screenWidth;
    const int m_screenHeight;

    Camera2D camera;

    const static Rectangle standardLeftBox;

    void UpdateControls();
    void UpdateCamera();
    void UpdateParticles() const;

    static void ShowParticles();
    static void ShowSettingsBox();
    static void ShowParametersBox();
};

enum class LeftHandSideGuiState{
    None,
    ParametersBox,
    SettingsBox
};

#endif
