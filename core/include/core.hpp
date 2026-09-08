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
        Vector3(real x, real y, real z);

        // Operators
        void operator*=(real value);
        void operator+=(const Vector3& vector);
        void operator-=(const Vector3& vector);
        // Cross product assignment
        void operator%=(const Vector3& vector);
        real operator[](unsigned i) const;
        real& operator[](unsigned i);
        Vector3 operator+(const Vector3& vector) const;
        Vector3 operator-(const Vector3& vector) const;
        real operator*(const Vector3& vector) const;
        // Dot product
        Vector3 operator*(const real& magnitude) const;
        // Cross product
        Vector3 operator%(const Vector3& vector) const;

        // Mutators
        void AddScaledVector(const Vector3& vector,  real scale);
        void ComponentProductUpdate(const Vector3& vector);
        void Clear();
        void Normalise();

        // Queries
        [[nodiscard]] Vector3 ComponentProduct(const Vector3& vector) const;
        [[nodiscard]] Vector3 VectorProduct(const Vector3& vector) const;
        [[nodiscard]] real ScalarProduct(const Vector3& vector) const;

        [[nodiscard]] real Magnitude() const;
        [[nodiscard]] real SquareMagnitude() const;

        // Static
        static void MakeOrthonormalBasis(Vector3* vectorA, Vector3* vectorB, Vector3* vectorC);
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
        Quaternion(real r, real i, real j, real k);

        // Operators
        void operator*= (const Quaternion& multiplier);

        // Mutators
        void AddScaledVector(const Vector3& vector3, real scale);
        void RotateByVector(const Vector3& vector3);
        void SetFromEuler(real x, real y, real z);
        void Normalise();
    };

    // A 3 x 3 Matrix
    // Stores a transformation in 3D space without a translation component.
    class Matrix3{
    public:
        real data[9]{};

        // Constructors
        Matrix3();
        Matrix3(const Matrix3& other);
        Matrix3(real a1, real a2, real a3, real b1, real b2, real b3, real c1, real c2, real c3);

        // Operators
        Matrix3 operator*(const Matrix3& other) const;
        void operator*= (const Matrix3& other);
        Vector3 operator*(const Vector3& vector3) const;

        // Mutators
        void SetInverse(const Matrix3& matrix);
        void SetTranspose(const Matrix3& matrix);
        void SetOrientation(const Quaternion& quaternion);
        void SetComponents(const Vector3& componentOne, const Vector3& componentTwo, const Vector3& componentThree);
        void Invert();

        // Queries
        [[nodiscard]] Matrix3 Inverse() const;
        [[nodiscard]] Matrix3 Transpose() const;
        [[nodiscard]] Vector3 Transform(const Vector3& vector3) const;
        [[nodiscard]] Vector3 TransformTranspose(const Vector3& vector3) const;

        // Static
        static Matrix3 LinearInterpolate(const Matrix3& startMatrix, const Matrix3& endMatrix, real proportion);
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
        Matrix4 operator*(const Matrix4 &other) const;
        Vector3 operator*(const Vector3& vector3) const;

        // Mutators
        void SetInverse(const Matrix4& matrix);
        void SetOrientationAndPos(const Quaternion& quaternion, const Vector3& position);
        void Invert();

        // Queries
        [[nodiscard]] real GetDeterminant() const;
        [[nodiscard]] Matrix4 Inverse() const;
        [[nodiscard]] Vector3 Transform(const Vector3& vector3) const;
        [[nodiscard]] Vector3 TransformInverse(const Vector3& vector3) const;
        [[nodiscard]] Vector3 TransformDirection(const Vector3& vector3) const;
        [[nodiscard]] Vector3 TransformInverseDirection(const Vector3& vector3) const;
        [[nodiscard]] Vector3 GetAxisVector(int index) const;
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
        static real RandomReal(real lowerBound, real upperbound);
        static int RandomInt(int lowerBound, int upperbound);
        static Vector3 RandomVector3(const Vector3 &lowerBound, const Vector3 &upperbound);
    };

    // Local and world transforms
    struct LocalWorldTransforms {
        static Vector3 LocalToWorld(const Vector3& local, const Matrix4& transform);
        static Vector3 WorldToLocal(const Vector3& world, const Matrix4& transform);

        static Vector3 LocalToWorldDirection(const Vector3& local, const Matrix4& transform);
        static Vector3 WorldToLocalDirection(const Vector3& world, const Matrix4& transform);
    };

    // Mathematical operations on real.
    real RealSqrt(real value);
    real RealPow(real value, real power);
    real RealAbs(real value);

}
