#pragma once
#include <random>

#include "precision.hpp"

namespace IPhysics
{
    // A 3D Vector
    struct Vector3
    {
        real x;
        real y;
        real z;

        // Constructors
        Vector3();
        Vector3(real _x, real _y, real _z);

        // Operators
        void operator*=(real _value);
        void operator+=(const Vector3& _vector);
        void operator-=(const Vector3& _vector);
        // Cross product assignment
        void operator%=(const Vector3& _vector);
        real operator[](unsigned i) const;
        real& operator[](unsigned i);
        Vector3 operator+(const Vector3& _vector) const;
        Vector3 operator-(const Vector3& _vector) const;
        real operator*(const Vector3& _vector) const;
        // Dot product
        Vector3 operator*(const real& _magnitude) const;
        // Cross product
        Vector3 operator%(const Vector3& _vector) const;

        // Mutators
        void AddScaledVector(const Vector3& _vector,  real scale);
        void ComponentProductUpdate(const Vector3& _vector);
        void Clear();
        void Normalise();

        // Queries
        [[nodiscard]] Vector3 ComponentProduct(const Vector3& _vector) const;
        [[nodiscard]] Vector3 VectorProduct(const Vector3& _vector) const;
        [[nodiscard]] real ScalarProduct(const Vector3& _vector) const;

        [[nodiscard]] real Magnitude() const;
        [[nodiscard]] real SquareMagnitude() const;

        // Static
        static void MakeOrthonormalBasis(Vector3* _vectorA, Vector3* _vectorB, Vector3* _vectorC);
    };

    // A Quaternion
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
            real data[4]{};
        };

        // Constructors
        Quaternion();
        Quaternion(real _r, real _i, real _j, real _k);

        // Operators
        void operator*= (const Quaternion& _multiplier);

        // Mutators
        void AddScaledVector(const Vector3& _vector3, real _scale);
        void RotateByVector(const Vector3& _vector3);
        void SetFromEuler(real _x, real _y, real _z);
        void Normalise();
    };

    // A 3 x 3 Matrix
    // Stores a transformation in 3D space without a translation component.
    class Matrix3{
    public:
        real data[9]{};

        // Constructors
        Matrix3();
        Matrix3(const Matrix3& _other);
        Matrix3(real _a1, real _a2, real _a3, real _b1, real _b2, real _b3, real _c1, real _c2, real _c3);

        // Operators
        Matrix3 operator*(const Matrix3& _other) const;
        void operator*= (const Matrix3& _other);
        Vector3 operator*(const Vector3& _vector3) const;

        // Mutators
        void SetInverse(const Matrix3& _matrix);
        void SetTranspose(const Matrix3& _matrix);
        void SetOrientation(const Quaternion& _quaternion);
        void SetComponents(const Vector3& _componentOne, const Vector3& _componentTwo, const Vector3& _componentThree);
        void Invert();

        // Queries
        [[nodiscard]] Matrix3 Inverse() const;
        [[nodiscard]] Matrix3 Transpose() const;
        [[nodiscard]] Vector3 Transform(const Vector3& _vector3) const;
        [[nodiscard]] Vector3 TransformTranspose(const Vector3& _vector3) const;

        // Static
        static Matrix3 LinearInterpolate(const Matrix3& _startMatrix, const Matrix3& _endMatrix, real _proportion);
    };

    // A 3 x 4 Matrix
    // Stores a transformation consisting of both a rotation and a position.
    // It assumes that the remaining four elements are (0,0,0,1), producing a homogenous matrix.
    class Matrix4{
    public:
        real data[12]{};

        // Constructors
        Matrix4();

        // Operators
        Matrix4 operator*(const Matrix4 &_other) const;
        Vector3 operator*(const Vector3& _vector3) const;

        // Mutators
        void SetInverse(const Matrix4& _matrix);
        void SetOrientationAndPos(const Quaternion& _quaternion, const Vector3& _position);
        void Invert();

        // Queries
        [[nodiscard]] real GetDeterminant() const;
        [[nodiscard]] Matrix4 Inverse() const;
        [[nodiscard]] Vector3 Transform(const Vector3& _vector3) const;
        [[nodiscard]] Vector3 TransformInverse(const Vector3& _vector3) const;
        [[nodiscard]] Vector3 TransformDirection(const Vector3& _vector3) const;
        [[nodiscard]] Vector3 TransformInverseDirection(const Vector3& _vector3) const;
        [[nodiscard]] Vector3 GetAxisVector(int _index) const;
    };

    // Constants
    const inline static Vector3 Origin(0,0,0);
    const inline static Vector3 Up(0,1,0);
    const inline static Vector3 Down(0,-1,0);
    const inline static Vector3 GravityEarth(0,-9.81f,0);

    // Random number generator
    struct RandomStore{
        static inline std::random_device randomDevice;
        static inline std::mt19937 generator{randomDevice()};;

        // Doesn't include upper bound.
        static real RandomReal(real _lowerBound, real _upperbound);
        static int RandomInt(int _lowerBound, int _upperbound);
        static Vector3 RandomVector3(const Vector3 &_lowerBound, const Vector3 &_upperbound);
    };

    // Local and world transforms
    struct LocalWorldTransforms {
        static Vector3 LocalToWorld(const Vector3& _local, const Matrix4& _transform);
        static Vector3 WorldToLocal(const Vector3& _world, const Matrix4& _transform);

        static Vector3 LocalToWorldDirection(const Vector3& _local, const Matrix4& _transform);
        static Vector3 WorldToLocalDirection(const Vector3& _world, const Matrix4& _transform);
    };

    // Mathematical operations on real.
    real RealSqrt(real _value);
    real RealPow(real _value, real _power);
    real RealAbs(real _value);

}
