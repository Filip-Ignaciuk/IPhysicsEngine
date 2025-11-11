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
        void SetX(real _x);
        void SetY(real _y);
        void SetZ(real _z);

        Vector3 ComponentProduct(const Vector3& _vector);


        void operator*=(const real _value);
        void operator+=(const Vector3& _vector);
        void operator-=(const Vector3& _vector);

        void operator%=(const Vector3& _vector);

        Vector3 operator+(const Vector3& _vector);
        Vector3 operator-(const Vector3& _vector);

        real operator*(const Vector3& _vector);
        Vector3 operator*(const real& _magnitude);
 
        Vector3 operator%(const Vector3& _vector);

    };

    class Quaternion{
        public:
        union{
            struct
            {
                real r;
                real i;
                real j;
                real k;
            };
            real data[4];
        };
        Quaternion();
        Quaternion(real _r, real _i, real _j, real _k);

        void Normalise();

        void operator*= (const Quaternion& _mulitplier);

        void RotateByVector(const Vector3& _vector3);

        void AddScaledVector(const Vector3& _vector3, real _scale);
    };

    class Matrix3{
    public:
        real data[9];

        Matrix3();

        Matrix3(real _a1, real _a2, real _a3, real _b1, real _b2, real _b3, real _c1, real _c2, real _c3);

        Matrix3 operator*(const Matrix3& _other) const;

        void operator*= (const Matrix3 &_other);

        void SetInverse(const Matrix3& _matrix);

        void SetTranspose(const Matrix3& _matrix);

        void SetOrientation(const Quaternion& _quaternion);

        Matrix3 Inverse() const;

        Matrix3 Transpose() const;

        void Invert();

        Vector3 operator*(const Vector3& _vector3) const;

        Vector3 Transform(const Vector3& _vector3) const;
    };

    class Matrix4{
    public:
        real data[12];



        Matrix4 operator*(const Matrix4 &_other) const;

        void SetInverse(const Matrix4& _matrix);

        void SetOrientationAndPos(const Quaternion& _quaternion, const Vector3& _position);

        real GetDeterminant() const;

        Matrix4 Inverse() const;

        void Invert();

        Vector3 operator*(const Vector3& _vector3) const;

        Vector3 Transform(const Vector3& _vector3) const;

        Vector3 TransformInverse(const Vector3& _vector3) const;

        Vector3 TransformDirection(const Vector3& _vector3) const;

        Vector3 TransformInverseDirection(const Vector3& _vector3) const;
    };

    const inline static Vector3 Origin(0,0,0);
    const inline static Vector3 Up(0,1,0);
    const inline static Vector3 Down(0,-1,0);
    const inline static Vector3 GravityEarth(0,-9.81f,0);


    struct RandomStore{
        static inline std::random_device randomDevice;
        static inline std::mt19937 generator;
        static void Initialise();
    };


    real RealSqrt(real _value);

    real RealPow(real _value, real _power);

    real RealAbs(real _value);

    Vector3 RandomVector3(Vector3 _lowerBound, Vector3 _upperbound);

    real RandomReal(real _lowerBound, real _upperbound);

    int RandomInt(int _lowerBound, int _upperbound);
}
