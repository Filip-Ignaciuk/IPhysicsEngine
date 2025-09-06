#pragma once
#include "precision.hpp"

namespace IPhysicsEngine
{
    class Vector3
    {
    private:
        real m_x;
        real m_y;
        real m_z;
    public:
        real Magnitude();
        real SquareMagnitude();
        void Normalise();

        Vector3();
        Vector3(real _x, real _y, real _z);
        ~Vector3();
    };
}
