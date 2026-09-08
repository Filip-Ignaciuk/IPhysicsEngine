#include "core.hpp"

/*
 * Vector
 */

// Constructors
IPhysics::Vector3::Vector3() : x(0), y(0), z(0){
};

IPhysics::Vector3::Vector3(real x, real y, real z) : x(x), y(y), z(z){

};

// Operators
void IPhysics::Vector3::operator*=(const real value){
    x *= value;
    y *= value;
    z *= value;
}

void IPhysics::Vector3::operator+=(const Vector3& vector){
    x += vector.x;
    y += vector.y;
    z += vector.z;
}

void IPhysics::Vector3::operator-=(const Vector3& vector){
    x -= vector.x;
    y -= vector.y;
    z -= vector.z;
}

void IPhysics::Vector3::operator%=(const Vector3& vector){
    *this = VectorProduct(vector);
}

IPhysics::real IPhysics::Vector3::operator[](unsigned i) const{
    if (i == 0) return x;
    if (i == 1) return y;
    return z;
}

IPhysics::real& IPhysics::Vector3::operator[](unsigned i){
    if (i == 0) return x;
    if (i == 1) return y;
    return z;
}

IPhysics::Vector3 IPhysics::Vector3::operator+(const Vector3& vector) const{
    return {x + vector.x, y + vector.y, z + vector.z};
}

IPhysics::Vector3 IPhysics::Vector3::operator-(const Vector3& vector) const{
    return {x - vector.x, y - vector.y, z - vector.z};
}

IPhysics::real IPhysics::Vector3::operator*(const Vector3& vector) const{
    return x * vector.x + y * vector.y + z * vector.z;
}

// Dot product
IPhysics::Vector3 IPhysics::Vector3::operator*(const IPhysics::real& magnitude) const{
    return {x * magnitude, y * magnitude, z * magnitude};
}

// Cross product
IPhysics::Vector3 IPhysics::Vector3::operator%(const Vector3& vector) const{
    return {y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x};
}

// Mutators
void IPhysics::Vector3::AddScaledVector(const Vector3& vector,  real scale){
    x += vector.x * scale;
    y += vector.y * scale;
    z += vector.z * scale;
};

void IPhysics::Vector3::ComponentProductUpdate(const Vector3& vector){
    x *= vector.x;
    y *= vector.y;
    z *= vector.z;
};

void IPhysics::Vector3::Clear(){
    x = 0;
    y = 0;
    z = 0;
}

void IPhysics::Vector3::Normalise(){
    const real magnitude = Magnitude();
    if (magnitude > 0){
        (*this) *= static_cast<real>(1) / magnitude;
    }
};

// Queries
IPhysics::Vector3 IPhysics::Vector3::ComponentProduct(const Vector3& vector) const{
    return {x * vector.x, y * vector.y, z * vector.z};
}

IPhysics::Vector3 IPhysics::Vector3::VectorProduct(const Vector3& vector) const{
    return {y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x};
}

IPhysics::real IPhysics::Vector3::ScalarProduct(const Vector3& vector) const{
    return x * vector.x + y * vector.y + z * vector.z;
}

IPhysics::real IPhysics::Vector3::Magnitude() const{
    return RealSqrt(x * x + y * y + z * z);
}

IPhysics::real IPhysics::Vector3::SquareMagnitude() const{
    return x * x + y * y + z * z;
};

// Static
void IPhysics::Vector3::MakeOrthonormalBasis(Vector3* vectorA, Vector3* vectorB, Vector3* vectorC){
    vectorA->Normalise();
    *vectorC = (*vectorA) % (*vectorB);
    if (vectorC->SquareMagnitude() == 0.0){
        return;
    }
    vectorC->Normalise();
    *vectorB = (*vectorC) % (*vectorA);
}

/*
 * Quaternion
 */

// Constructors
IPhysics::Quaternion::Quaternion() : r(0.0f), i(0.0f), j(0.0f), k(0.0f){
}

IPhysics::Quaternion::Quaternion(real r, real i, real j, real k) : r(r), i(i), j(j), k(k){

}

// Operators
void IPhysics::Quaternion::operator*= (const Quaternion& multiplier){
    Quaternion quaternion = *this;
    r = quaternion.r*multiplier.r - quaternion.i*multiplier.i -
    quaternion.j*multiplier.j - quaternion.k*multiplier.k;
    i = quaternion.r*multiplier.i + quaternion.i*multiplier.r +
    quaternion.j*multiplier.k - quaternion.k*multiplier.j;
    j = quaternion.r*multiplier.j + quaternion.j*multiplier.r +
    quaternion.k*multiplier.i - quaternion.i*multiplier.k;
    k = quaternion.r*multiplier.k + quaternion.k*multiplier.r +
    quaternion.i*multiplier.j - quaternion.j*multiplier.i;
}

// Mutators
void IPhysics::Quaternion::AddScaledVector(const Vector3& vector3, real scale){
    Quaternion q{
        0,
        vector3.x * scale,
        vector3.y * scale,
        vector3.z * scale
    };
    constexpr real HALF = static_cast<real>(0.5);
    q *= *this;
    r += q.r * HALF;
    i += q.i * HALF;
    j += q.j * HALF;
    k += q.k * HALF;
}

void IPhysics::Quaternion::RotateByVector(const Vector3& vector3){
    const Quaternion q{0, vector3.x, vector3.y, vector3.z};
    (*this) *= q;
}

void IPhysics::Quaternion::SetFromEuler(real x, real y, real z) {
    // Setting it initially to an identity quaternion
    r = 1;
    i = 0;
    j = 0;
    k = 0;

    const Quaternion q1{RealCos(z/2), 0 , 0 , RealSin(z/2)};
    const Quaternion q2{RealCos(y/2), 0 , RealSin(y/2) , 0};
    const Quaternion q3{RealCos(x/2), RealSin(x/2) , 0 , 0};

    (*this) *= q1;
    (*this) *= q2;
    (*this) *= q3;
}

void IPhysics::Quaternion::Normalise(){
    real d = r*r+i*i+j*j+k*k;

    if (d == 0){
        r = 1;
        return;
    }

    d = ((real)1.0)/RealSqrt(d);
    r *= d;
    i *= d;
    j *= d;
    k *= d;
}

/*
 * Matrix3
 */

// Constructors
IPhysics::Matrix3::Matrix3(){
    for (int i = 0; i < 9; i++)
    {
        data[i] = 0.0f;
    }
    
}

IPhysics::Matrix3::Matrix3(const Matrix3& other){
    for (int i = 0; i < 9; i++)
    {
        data[i] = other.data[i];
    }
}

IPhysics::Matrix3::Matrix3(real a1, real a2, real a3, real b1, real b2, real b3, real c1, real c2, real c3){
    data[0] = a1;
    data[1] = a2;
    data[2] = a3;
    data[3] = b1;
    data[4] = b2;
    data[5] = b3;
    data[6] = c1;
    data[7] = c2;
    data[8] = c3;
}

// Operators
IPhysics::Matrix3 IPhysics::Matrix3::operator*(const Matrix3& other) const{
    return {
        data[0]*other.data[0] + data[1]*other.data[3] + data[2]*other.data[6],
        data[0]*other.data[1] + data[1]*other.data[4] + data[2]*other.data[7],
        data[0]*other.data[2] + data[1]*other.data[5] + data[2]*other.data[8],
        data[3]*other.data[0] + data[4]*other.data[3] + data[5]*other.data[6],
        data[3]*other.data[1] + data[4]*other.data[4] + data[5]*other.data[7],
        data[3]*other.data[2] + data[4]*other.data[5] + data[5]*other.data[8],
        data[6]*other.data[0] + data[7]*other.data[3] + data[8]*other.data[6],
        data[6]*other.data[1] + data[7]*other.data[4] + data[8]*other.data[7],
        data[6]*other.data[2] + data[7]*other.data[5] + data[8]*other.data[8]
    };
}

void IPhysics::Matrix3::operator*=(const Matrix3& other){
    real t1 = data[0] * other.data[0] + data[1] * other.data[3] + data[2] * other.data[6];
    real t2 = data[0] * other.data[1] + data[1] * other.data[4] + data[2] * other.data[7];
    real t3 = data[0] * other.data[2] + data[1] * other.data[5] + data[2] * other.data[8];
    data[0] = t1;
    data[1] = t2;
    data[2] = t3;
    t1 = data[3]*other.data[0] + data[4]*other.data[3] + data[5]*other.data[6];
    t2 = data[3]*other.data[1] + data[4]*other.data[4] + data[5]*other.data[7];
    t3 = data[3]*other.data[2] + data[4]*other.data[5] + data[5]*other.data[8];
    data[3] = t1;
    data[4] = t2;
    data[5] = t3;
    t1 = data[6]*other.data[0] + data[7]*other.data[3] + data[8]*other.data[6];
    t2 = data[6]*other.data[1] + data[7]*other.data[4] + data[8]*other.data[7];
    t3 = data[6]*other.data[2] + data[7]*other.data[5] + data[8]*other.data[8];
    data[6] = t1;
    data[7] = t2;
    data[8] = t3;
}

IPhysics::Vector3 IPhysics::Matrix3::operator*(const Vector3& vector3) const{
    return {
        vector3.x * data[0] + vector3.y * data[1] + vector3.z * data[2],
        vector3.x * data[3] + vector3.y * data[4] + vector3.z * data[5],
        vector3.x * data[6] + vector3.y * data[7] + vector3.z * data[8]
    };
}

// Mutators
void IPhysics::Matrix3::SetInverse(const Matrix3& matrix){
    const real t1 = matrix.data[0]*matrix.data[4];
    const real t2 = matrix.data[0]*matrix.data[5];
    const real t3 = matrix.data[1]*matrix.data[3];
    const real t4 = matrix.data[2]*matrix.data[3];
    const real t5 = matrix.data[1]*matrix.data[6];
    const real t6 = matrix.data[2]*matrix.data[6];
    const real det = (t1*matrix.data[8] - t2*matrix.data[7] - t3*matrix.data[8]+
    t4*matrix.data[7] + t5*matrix.data[5] - t6*matrix.data[4]);
    if (det == static_cast<real>(0.0f)) return;
    real inverse = static_cast<real>(1.0f)/det;

    data[0] = (matrix.data[4]*matrix.data[8]-matrix.data[5]*matrix.data[7])*inverse;
    data[1] = -(matrix.data[1]*matrix.data[8]-matrix.data[2]*matrix.data[7])*inverse;
    data[2] = (matrix.data[1]*matrix.data[5]-matrix.data[2]*matrix.data[4])*inverse;
    data[3] = -(matrix.data[3]*matrix.data[8]-matrix.data[5]*matrix.data[6])*inverse;
    data[4] = (matrix.data[0]*matrix.data[8]-t6)*inverse;
    data[5] = -(t2-t4)*inverse;
    data[6] = (matrix.data[3]*matrix.data[7]-matrix.data[4]*matrix.data[6])*inverse;
    data[7] = -(matrix.data[0]*matrix.data[7]-t5)*inverse;
    data[8] = (t1-t3)*inverse;
}

void IPhysics::Matrix3::SetTranspose(const Matrix3& matrix){
    data[0] = matrix.data[0];
    data[1] = matrix.data[3];
    data[2] = matrix.data[6];
    data[3] = matrix.data[1];
    data[4] = matrix.data[4];
    data[5] = matrix.data[7];
    data[6] = matrix.data[2];
    data[7] = matrix.data[5];
    data[8] = matrix.data[8];
}

void IPhysics::Matrix3::SetOrientation(const Quaternion& quaternion){
    data[0] = 1 - (2*quaternion.j*quaternion.j + 2*quaternion.k*quaternion.k);
    data[1] = 2*quaternion.i*quaternion.j + 2*quaternion.k*quaternion.r;
    data[2] = 2*quaternion.i*quaternion.k - 2*quaternion.j*quaternion.r;
    data[3] = 2*quaternion.i*quaternion.j - 2*quaternion.k*quaternion.r;
    data[4] = 1 - (2*quaternion.i*quaternion.i + 2*quaternion.k*quaternion.k);
    data[5] = 2*quaternion.j*quaternion.k + 2*quaternion.i*quaternion.r;
    data[6] = 2*quaternion.i*quaternion.k + 2*quaternion.j*quaternion.r;
    data[7] = 2*quaternion.j*quaternion.k - 2*quaternion.i*quaternion.r;
    data[8] = 1 - (2*quaternion.i*quaternion.i + 2*quaternion.j*quaternion.j);
}

void IPhysics::Matrix3::SetComponents(const Vector3& componentOne, const Vector3& componentTwo, const Vector3& componentThree){
    data[0] = componentOne.x;
    data[1] = componentTwo.x;
    data[2] = componentThree.x;
    data[3] = componentOne.y;
    data[4] = componentTwo.y;
    data[5] = componentThree.y;
    data[6] = componentOne.z;
    data[7] = componentTwo.z;
    data[8] = componentThree.z;
}

void IPhysics::Matrix3::Invert(){
    SetInverse(*this);
}

// Queries
IPhysics::Matrix3 IPhysics::Matrix3::Inverse() const{
    Matrix3 result;
    result.SetInverse(*this);
    return result;
}

IPhysics::Matrix3 IPhysics::Matrix3::Transpose() const{
    Matrix3 result;
    result.SetTranspose(*this);
    return result;
}

IPhysics::Vector3 IPhysics::Matrix3::Transform(const Vector3& vector3) const{
    return (*this) * vector3;
}

IPhysics::Vector3 IPhysics::Matrix3::TransformTranspose(const Vector3& vector3) const{
    return {
        vector3.x * data[0] + vector3.y * data[3] + vector3.z * data[6],
        vector3.x * data[1] + vector3.y * data[4] + vector3.z * data[7],
        vector3.x * data[2] + vector3.y * data[5] + vector3.z * data[8]
    };
}

// Static
IPhysics::Matrix3 IPhysics::Matrix3::LinearInterpolate(const Matrix3& startMatrix, const Matrix3& endMatrix, real proportion){
    Matrix3 result;
    real omp = 1.0f - proportion;
    for (unsigned i = 0; i < 9; i++){
        result.data[i] = startMatrix.data[i] * omp + endMatrix.data[i] * proportion;
    }
    return result;
}

/*
 * Matrix4
 */

// Constructors
IPhysics::Matrix4::Matrix4() = default;

// Operators
IPhysics::Matrix4 IPhysics::Matrix4::operator*(const Matrix4& other) const{
    Matrix4 result{};
    result.data[0] = other.data[0]*data[0] + other.data[4]*data[1] +
    other.data[8]*data[2];
    result.data[4] = other.data[0]*data[4] + other.data[4]*data[5] +
    other.data[8]*data[6];
    result.data[8] = other.data[0]*data[8] + other.data[4]*data[9] +
    other.data[8]*data[10];
    result.data[1] = other.data[1]*data[0] + other.data[5]*data[1] +
    other.data[9]*data[2];
    result.data[5] = other.data[1]*data[4] + other.data[5]*data[5] +
    other.data[9]*data[6];
    result.data[9] = other.data[1]*data[8] + other.data[5]*data[9] +
    other.data[9]*data[10];
    result.data[2] = other.data[2]*data[0] + other.data[6]*data[1] +
    other.data[10]*data[2];
    result.data[6] = other.data[2]*data[4] + other.data[6]*data[5] +
    other.data[10]*data[6];
    result.data[10] = other.data[2]*data[8] + other.data[6]*data[9] +
    other.data[10]*data[10];
    result.data[3] = other.data[3]*data[0] + other.data[7]*data[1] +
    other.data[11]*data[2] + data[3];
    result.data[7] = other.data[3]*data[4] + other.data[7]*data[5] +
    other.data[11]*data[6] + data[7];
    result.data[11] = other.data[3]*data[8] + other.data[7]*data[9] +
    other.data[11]*data[10] + data[11];
    return result;
}

IPhysics::Vector3 IPhysics::Matrix4::operator*(const Vector3& vector3) const{
    return {
        vector3.x * data[0] + vector3.y * data[1] + vector3.z * data[2] + data[3],
        vector3.x * data[4] + vector3.y * data[5] + vector3.z * data[6] + data[7],
        vector3.x * data[8] + vector3.y * data[9] + vector3.z * data[10] + data[11]
    };
}

// Mutators
void IPhysics::Matrix4::SetInverse(const Matrix4& matrix){
    real det = GetDeterminant();
    if (det == 0) return;
    det = static_cast<real>(1.0f)/det;
    data[0] = (-matrix.data[9]*matrix.data[6]+matrix.data[5]*matrix.data[10])*det;
    data[4] = (matrix.data[8]*matrix.data[6]-matrix.data[4]*matrix.data[10])*det;
    data[8] = (-matrix.data[8]*matrix.data[5]+matrix.data[4]*matrix.data[9]*matrix.data[15])*det;
    data[1] = (matrix.data[9]*matrix.data[2]-matrix.data[1]*matrix.data[10])*det;
    data[5] = (-matrix.data[8]*matrix.data[2]+matrix.data[0]*matrix.data[10])*det;
    data[9] = (matrix.data[8]*matrix.data[1]-matrix.data[0]*matrix.data[9]*matrix.data[15])*det;
    data[2] = (-matrix.data[5]*matrix.data[2]+matrix.data[1]*matrix.data[6]*matrix.data[15])*det;
    data[6] = (+matrix.data[4]*matrix.data[2]-matrix.data[0]*matrix.data[6]*matrix.data[15])*det;
    data[10] = (-matrix.data[4]*matrix.data[1]+matrix.data[0]*matrix.data[5]*matrix.data[15])*det;
    data[3] = (matrix.data[9]*matrix.data[6]*matrix.data[3]
    -matrix.data[5]*matrix.data[10]*matrix.data[3]
    -matrix.data[9]*matrix.data[2]*matrix.data[7]
    +matrix.data[1]*matrix.data[10]*matrix.data[7]
    +matrix.data[5]*matrix.data[2]*matrix.data[11]
    -matrix.data[1]*matrix.data[6]*matrix.data[11])*det;
    data[7] = (-matrix.data[8]*matrix.data[6]*matrix.data[3]
    +matrix.data[4]*matrix.data[10]*matrix.data[3]
    +matrix.data[8]*matrix.data[2]*matrix.data[7]
    -matrix.data[0]*matrix.data[10]*matrix.data[7]
    -matrix.data[4]*matrix.data[2]*matrix.data[11]
    +matrix.data[0]*matrix.data[6]*matrix.data[11])*det;
    data[11] =(matrix.data[8]*matrix.data[5]*matrix.data[3]
    -matrix.data[4]*matrix.data[9]*matrix.data[3]
    -matrix.data[8]*matrix.data[1]*matrix.data[7]
    +matrix.data[0]*matrix.data[9]*matrix.data[7]
    +matrix.data[4]*matrix.data[1]*matrix.data[11]
    -matrix.data[0]*matrix.data[5]*matrix.data[11])*det;
}

void IPhysics::Matrix4::SetOrientationAndPos(const Quaternion& quaternion, const Vector3& position){
    data[0] = 1 - (2*quaternion.j*quaternion.j + 2*quaternion.k*quaternion.k);
    data[1] = 2*quaternion.i*quaternion.j + 2*quaternion.k*quaternion.r;
    data[2] = 2*quaternion.i*quaternion.k - 2*quaternion.j*quaternion.r;
    data[3] = position.x;
    data[4] = 2*quaternion.i*quaternion.j - 2*quaternion.k*quaternion.r;
    data[5] = 1 - (2*quaternion.i*quaternion.i + 2*quaternion.k*quaternion.k);
    data[6] = 2*quaternion.j*quaternion.k + 2*quaternion.i*quaternion.r;
    data[7] = position.y;
    data[8] = 2*quaternion.i*quaternion.k + 2*quaternion.j*quaternion.r;
    data[9] = 2*quaternion.j*quaternion.k - 2*quaternion.i*quaternion.r;
    data[10] = 1 - (2*quaternion.i*quaternion.i + 2*quaternion.j*quaternion.j);
    data[11] = position.z;
}

void IPhysics::Matrix4::Invert(){
    SetInverse(*this);
}

// Queries
IPhysics::real IPhysics::Matrix4::GetDeterminant() const{
    return  data[8]*data[5]*data[2]+
            data[4]*data[9]*data[2]+
            data[8]*data[1]*data[6]-
            data[0]*data[9]*data[6]-
            data[4]*data[1]*data[10]+
            data[0]*data[5]*data[10];
}

IPhysics::Matrix4 IPhysics::Matrix4::Inverse() const{
    Matrix4 result;
    result.SetInverse(*this);
    return result;
}

IPhysics::Vector3 IPhysics::Matrix4::Transform(const Vector3& vector3) const{
    return (*this) * vector3;
}

IPhysics::Vector3 IPhysics::Matrix4::TransformInverse(const Vector3& vector3) const{
    Vector3 tmp = vector3;
    tmp.x = tmp.x - data[3];
    tmp.y = tmp.y - data[7];
    tmp.z = tmp.z -= data[11];
    return {
        tmp.x * data[0] +
        tmp.y * data[4] +
        tmp.z * data[8],
        tmp.x * data[1] +
        tmp.y * data[5] +
        tmp.z * data[9],
        tmp.x * data[2] +
        tmp.y * data[6] +
        tmp.z * data[10]
    };
}

IPhysics::Vector3 IPhysics::Matrix4::TransformDirection(const Vector3& vector3) const{
    return {
        vector3.x * data[0] +
        vector3.y * data[1] +
        vector3.z * data[2],
        vector3.x * data[4] +
        vector3.y * data[5] +
        vector3.z * data[6],
        vector3.x * data[8] +
        vector3.y * data[9] +
        vector3.z * data[10]
    };
}

IPhysics::Vector3 IPhysics::Matrix4::TransformInverseDirection(const Vector3& vector3) const{
    return {
        vector3.x * data[0] +
        vector3.y * data[4] +
        vector3.z * data[8],
        vector3.x * data[1] +
        vector3.y * data[5] +
        vector3.z * data[9],
        vector3.x * data[2] +
        vector3.y * data[6] +
        vector3.z * data[10]
    };
}

IPhysics::Vector3 IPhysics::Matrix4::GetAxisVector(int index) const{
    return {data[index], data[index + 4], data[index + 8]};
}

/*
 * Random number generator
 */
IPhysics::real IPhysics::RandomStore::RandomReal(real lowerBound, real upperbound){
    std::uniform_real_distribution<real> doubleDistribution(lowerBound, upperbound);
    return doubleDistribution(RandomStore::generator);
}

int IPhysics::RandomStore::RandomInt(int lowerBound, int upperbound){
    std::uniform_int_distribution<int> intDistribution(lowerBound, upperbound);
    return intDistribution(RandomStore::generator);
}

IPhysics::Vector3 IPhysics::RandomStore::RandomVector3(const Vector3 &lowerBound, const Vector3 &upperbound){
    return {RandomReal(lowerBound.x, upperbound.x), RandomReal(lowerBound.y, upperbound.y), RandomReal(lowerBound.z, upperbound.z)};
}

/*
 * Local and world transforms
 */
IPhysics::Vector3 IPhysics::LocalWorldTransforms::LocalToWorld(const Vector3& local, const Matrix4& transform){
    return transform.Transform(local);
}

IPhysics::Vector3 IPhysics::LocalWorldTransforms::WorldToLocal(const Vector3& world, const Matrix4& transform){
    return transform.TransformInverse(world);
}

IPhysics::Vector3 IPhysics::LocalWorldTransforms::LocalToWorldDirection(const Vector3& local, const Matrix4& transform){
    return transform.TransformDirection(local);
}

IPhysics::Vector3 IPhysics::LocalWorldTransforms::WorldToLocalDirection(const Vector3& world, const Matrix4& transform){
    return transform.TransformInverseDirection(world);
}

/*
 *  Mathematical operations on real.
 */
IPhysics::real IPhysics::RealSqrt(real value){
    return std::sqrt(value);
}

IPhysics::real IPhysics::RealPow(real value, real power){
    return std::pow(value, power);
}

IPhysics::real IPhysics::RealAbs(real value){
    return std::abs(value);
}
