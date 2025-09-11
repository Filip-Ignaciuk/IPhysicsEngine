#pragma once
#include <cmath>
#include <random>


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

        void Normalise();
        void AddScaledVector(const Vector3& _vector,  real scale);
        void ComponentProductUpdate(const Vector3& _vector);
        real ScalarProduct(const Vector3& _vector);
        Vector3 VectorProduct(const Vector3& _vector);
        void Clear();

        void MakeOrthonormalBasis(Vector3* _vectorA, Vector3* _vectorB, Vector3* _vectorC);


        Vector3();
        Vector3(real _x, real _y, real _z);
        ~Vector3();

        real Magnitude() const;
        real SquareMagnitude() const;
        real GetX() const;
        real GetY() const;
        real GetZ() const;

        Vector3 ComponentProduct(const Vector3& _vector);


        void operator*=(const real _value);
        void operator+=(const Vector3& _vector);
        void operator-=(const Vector3& _vector);

        void operator%=(const Vector3& _vector);

        Vector3 operator+(const Vector3& _vector);
        Vector3 operator-(const Vector3& _vector);

        real operator*(const Vector3& _vector);

        Vector3 operator%(const Vector3& _vector);

    };

    const inline static Vector3 Origin(0,0,0);
    const inline static Vector3 Gravity(0,-9.81f,0);


    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());


    real RealSqrt(real _value);

    real RealPow(real _value, real _power);

    Vector3 RandomVector3(real _lowerBound, real _upperbound);

    real RandomReal(real _lowerBound, real _upperbound);

    int RandomInt(int _lowerBound, int _upperbound);

}
