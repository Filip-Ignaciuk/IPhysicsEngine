#include "igravityview.hpp"

#include <cmath>

#include "raylib.h"

#include "raymath.h"

IGravityView::IGravityView(IGravityModel* _iGravityModel,
    IGravityController* _iGravityController,
    int _screenWidth,
    int _screenHeight) :
    m_iGravityModel(_iGravityModel),
    m_iGravityController(_iGravityController),
    m_screenWidth(_screenWidth),
    m_screenHeight(_screenHeight)
{
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

void IGravityView::UpdateControls(){
    if(IsKeyPressed(KEY_SPACE)){
        m_iGravityModel->SetSimulationPause(!m_iGravityModel->IsSimulationPaused());
    }
}

void IGravityView::UpdateCamera() {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        const Vector2 delta = Vector2Scale(
            m_iGravityController->GetLatestMouseDelta(), -1.0f/camera.zoom);
        camera.target = Vector2Add(camera.target, delta);
    }

    if (m_iGravityController->GetLatestMouseWheelMove() != 0) {
        Vector2 mouseWorldPos = GetScreenToWorld2D(
            m_iGravityController->GetLatestMousePosition(), camera);
        camera.offset = m_iGravityController->GetLatestMousePosition();
        camera.target = mouseWorldPos;
        float scale = 0.2f * m_iGravityController->GetLatestMouseWheelMove();
        camera.zoom = Clamp(
            expf(logf(camera.zoom) + scale), 0.125f, 64.0f);
    }
}

void IGravityView::UpdateParticles() const {
    for (int i = 0; i < m_iGravityModel->GetParticles().size(); ++i) {
        IPhysics::Object* obj = m_iGravityModel->GetParticles()[i];
        const auto* rigidbody = obj->GetComponent<IPhysics::RigidBody>();
        const Vector2 position{
            .x = static_cast<float>(rigidbody->GetPosition().x),
            .y = static_cast<float>(rigidbody->GetPosition().y)
        };
        DrawCircleV(position, 1.0f, RED);
    }
}
