#include "igravitycontroller.hpp"

#include "raylib.h"

IGravityController::IGravityController() {

}

void IGravityController::Update() {
    m_latestWheelMove = GetMouseWheelMove();
    m_latestMousePosition = GetMousePosition();
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        m_latestMouseDelta = GetMouseDelta();
    }
    else {
        m_latestMouseDelta = {};
    }
}

float IGravityController::GetLatestMouseWheelMove() const {
    return m_latestWheelMove;
}

const Vector2& IGravityController::GetLatestMousePosition() const {
    return m_latestMousePosition;
}

const Vector2& IGravityController::GetLatestMouseDelta() const {
    return m_latestMouseDelta;
}
