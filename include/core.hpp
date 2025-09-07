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
        void AddScaledVector(const Vector3& _vector,  real scale);
        void ComponentProductUpdate(const Vector3& _vector);
        real ScalarProduct(const Vector3& _vector);

        Vector3();
        Vector3(real _x, real _y, real _z);
        ~Vector3();

        Vector3 ComponentProduct(const Vector3& _vector);


        void operator*=(const real _value);
        void operator+=(const Vector3& _vector);
        void operator-=(const Vector3& _vector);

        Vector3 operator+(const Vector3& _vector);
        Vector3 operator-(const Vector3& _vector);
        real operator*(const Vector3& _vector);

    };
}
