#ifndef IPHYSICS_IGRAVITYCONTROLLER_HPP
#define IPHYSICS_IGRAVITYCONTROLLER_HPP
#include <raylib.h>

class IGravityController {
    public:
    IGravityController();

    void Update();

    [[nodiscard]] float GetLatestMouseWheelMove() const;
    [[nodiscard]] const Vector2& GetLatestMousePosition() const;
    [[nodiscard]] const Vector2& GetLatestMouseDelta() const;

private:
    float m_latestWheelMove;
    Vector2 m_latestMousePosition;
    Vector2 m_latestMouseDelta;
};

#endif
