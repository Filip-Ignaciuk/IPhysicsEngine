#include "core.hpp"

/*
 * Vector
 */

// Constructors
IPhysics::Vector3::Vector3() : x(0), y(0), z(0){
};

IPhysics::Vector3::Vector3(real _x, real _y, real _z) : x(_x), y(_y), z(_z){

};

// Operators
void IPhysics::Vector3::operator*=(const real _value){
    x *= _value;
    y *= _value;
    z *= _value;
}

void IPhysics::Vector3::operator+=(const Vector3& _vector){
    x += _vector.x;
    y += _vector.y;
    z += _vector.z;
}

void IPhysics::Vector3::operator-=(const Vector3& _vector){
    x -= _vector.x;
    y -= _vector.y;
    z -= _vector.z;
}

void IPhysics::Vector3::operator%=(const Vector3& _vector){
    *this = VectorProduct(_vector);
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

IPhysics::Vector3 IPhysics::Vector3::operator+(const Vector3& _vector) const{
    return {x + _vector.x, y + _vector.y, z + _vector.z};
}

IPhysics::Vector3 IPhysics::Vector3::operator-(const Vector3& _vector) const{
    return {x - _vector.x, y - _vector.y, z - _vector.z};
}

IPhysics::real IPhysics::Vector3::operator*(const Vector3& _vector) const{
    return x * _vector.x + y * _vector.y + z * _vector.z;
}

// Dot product
IPhysics::Vector3 IPhysics::Vector3::operator*(const IPhysics::real& _magnitude) const{
    return {x * _magnitude, y * _magnitude, z * _magnitude};
}

// Cross product
IPhysics::Vector3 IPhysics::Vector3::operator%(const Vector3& _vector) const{
    return {y * _vector.z - z * _vector.y, z * _vector.x - x * _vector.z, x * _vector.y - y * _vector.x};
}

// Mutators
void IPhysics::Vector3::AddScaledVector(const Vector3& _vector,  real scale){
    x += _vector.x * scale;
    y += _vector.y * scale;
    z += _vector.z * scale;
};

void IPhysics::Vector3::ComponentProductUpdate(const Vector3& _vector){
    x *= _vector.x;
    y *= _vector.y;
    z *= _vector.z;
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
IPhysics::Vector3 IPhysics::Vector3::ComponentProduct(const Vector3& _vector) const{
    return {x * _vector.x, y * _vector.y, z * _vector.z};
}

IPhysics::Vector3 IPhysics::Vector3::VectorProduct(const Vector3& _vector) const{
    return {y * _vector.z - z * _vector.y, z * _vector.x - x * _vector.z, x * _vector.y - y * _vector.x};
}

IPhysics::real IPhysics::Vector3::ScalarProduct(const Vector3& _vector) const{
    return x * _vector.x + y * _vector.y + z * _vector.z;
}

IPhysics::real IPhysics::Vector3::Magnitude() const{
    return RealSqrt(x * x + y * y + z * z);
}

IPhysics::real IPhysics::Vector3::SquareMagnitude() const{
    return x * x + y * y + z * z;
};

// Static
void IPhysics::Vector3::MakeOrthonormalBasis(Vector3* _vectorA, Vector3* _vectorB, Vector3* _vectorC){
    _vectorA->Normalise();
    *_vectorC = (*_vectorA) % (*_vectorB);
    if (_vectorC->SquareMagnitude() == 0.0){
        return;
    }
    _vectorC->Normalise();
    *_vectorB = (*_vectorC) % (*_vectorA);
}

/*
 * Quaternion
 */

// Constructors
IPhysics::Quaternion::Quaternion() : r(0.0f), i(0.0f), j(0.0f), k(0.0f){
}

IPhysics::Quaternion::Quaternion(real _r, real _i, real _j, real _k) : r(_r), i(_i), j(_j), k(_k){

}

// Operators
void IPhysics::Quaternion::operator*= (const Quaternion& _multiplier){
    Quaternion quaternion = *this;
    r = quaternion.r*_multiplier.r - quaternion.i*_multiplier.i -
    quaternion.j*_multiplier.j - quaternion.k*_multiplier.k;
    i = quaternion.r*_multiplier.i + quaternion.i*_multiplier.r +
    quaternion.j*_multiplier.k - quaternion.k*_multiplier.j;
    j = quaternion.r*_multiplier.j + quaternion.j*_multiplier.r +
    quaternion.k*_multiplier.i - quaternion.i*_multiplier.k;
    k = quaternion.r*_multiplier.k + quaternion.k*_multiplier.r +
    quaternion.i*_multiplier.j - quaternion.j*_multiplier.i;
}

// Mutators
void IPhysics::Quaternion::AddScaledVector(const Vector3& _vector3, real _scale){
    Quaternion q{
        0,
        _vector3.x * _scale,
        _vector3.y * _scale,
        _vector3.z * _scale
    };
    constexpr real HALF = static_cast<real>(0.5);
    q *= *this;
    r += q.r * HALF;
    i += q.i * HALF;
    j += q.j * HALF;
    k += q.k * HALF;
}

void IPhysics::Quaternion::RotateByVector(const Vector3& _vector3){
    const Quaternion q{0, _vector3.x, _vector3.y, _vector3.z};
    (*this) *= q;
}

void IPhysics::Quaternion::SetFromEuler(real _x, real _y, real _z) {
    // Setting it initially to an identity quaternion
    r = 1;
    i = 0;
    j = 0;
    k = 0;

    const Quaternion q1{RealCos(_z/2), 0 , 0 , RealSin(_z/2)};
    const Quaternion q2{RealCos(_y/2), 0 , RealSin(_y/2) , 0};
    const Quaternion q3{RealCos(_x/2), RealSin(_x/2) , 0 , 0};

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

IPhysics::Matrix3::Matrix3(const Matrix3& _other){
    for (int i = 0; i < 9; i++)
    {
        data[i] = _other.data[i];
    }
}

IPhysics::Matrix3::Matrix3(real _a1, real _a2, real _a3, real _b1, real _b2, real _b3, real _c1, real _c2, real _c3){
    data[0] = _a1;
    data[1] = _a2;
    data[2] = _a3;
    data[3] = _b1;
    data[4] = _b2;
    data[5] = _b3;
    data[6] = _c1;
    data[7] = _c2;
    data[8] = _c3;
}

// Operators
IPhysics::Matrix3 IPhysics::Matrix3::operator*(const Matrix3& _other) const{
    return {
        data[0]*_other.data[0] + data[1]*_other.data[3] + data[2]*_other.data[6],
        data[0]*_other.data[1] + data[1]*_other.data[4] + data[2]*_other.data[7],
        data[0]*_other.data[2] + data[1]*_other.data[5] + data[2]*_other.data[8],
        data[3]*_other.data[0] + data[4]*_other.data[3] + data[5]*_other.data[6],
        data[3]*_other.data[1] + data[4]*_other.data[4] + data[5]*_other.data[7],
        data[3]*_other.data[2] + data[4]*_other.data[5] + data[5]*_other.data[8],
        data[6]*_other.data[0] + data[7]*_other.data[3] + data[8]*_other.data[6],
        data[6]*_other.data[1] + data[7]*_other.data[4] + data[8]*_other.data[7],
        data[6]*_other.data[2] + data[7]*_other.data[5] + data[8]*_other.data[8]
    };
}

void IPhysics::Matrix3::operator*=(const Matrix3& _other){
    real t1 = data[0] * _other.data[0] + data[1] * _other.data[3] + data[2] * _other.data[6];
    real t2 = data[0] * _other.data[1] + data[1] * _other.data[4] + data[2] * _other.data[7];
    real t3 = data[0] * _other.data[2] + data[1] * _other.data[5] + data[2] * _other.data[8];
    data[0] = t1;
    data[1] = t2;
    data[2] = t3;
    t1 = data[3]*_other.data[0] + data[4]*_other.data[3] + data[5]*_other.data[6];
    t2 = data[3]*_other.data[1] + data[4]*_other.data[4] + data[5]*_other.data[7];
    t3 = data[3]*_other.data[2] + data[4]*_other.data[5] + data[5]*_other.data[8];
    data[3] = t1;
    data[4] = t2;
    data[5] = t3;
    t1 = data[6]*_other.data[0] + data[7]*_other.data[3] + data[8]*_other.data[6];
    t2 = data[6]*_other.data[1] + data[7]*_other.data[4] + data[8]*_other.data[7];
    t3 = data[6]*_other.data[2] + data[7]*_other.data[5] + data[8]*_other.data[8];
    data[6] = t1;
    data[7] = t2;
    data[8] = t3;
}

IPhysics::Vector3 IPhysics::Matrix3::operator*(const Vector3& _vector3) const{
    return {
        _vector3.x * data[0] + _vector3.y * data[1] + _vector3.z * data[2],
        _vector3.x * data[3] + _vector3.y * data[4] + _vector3.z * data[5],
        _vector3.x * data[6] + _vector3.y * data[7] + _vector3.z * data[8]
    };
}

// Mutators
void IPhysics::Matrix3::SetInverse(const Matrix3& _matrix){
    const real t1 = _matrix.data[0]*_matrix.data[4];
    const real t2 = _matrix.data[0]*_matrix.data[5];
    const real t3 = _matrix.data[1]*_matrix.data[3];
    const real t4 = _matrix.data[2]*_matrix.data[3];
    const real t5 = _matrix.data[1]*_matrix.data[6];
    const real t6 = _matrix.data[2]*_matrix.data[6];
    const real det = (t1*_matrix.data[8] - t2*_matrix.data[7] - t3*_matrix.data[8]+
    t4*_matrix.data[7] + t5*_matrix.data[5] - t6*_matrix.data[4]);
    if (det == static_cast<real>(0.0f)) return;
    real inverse = static_cast<real>(1.0f)/det;

    data[0] = (_matrix.data[4]*_matrix.data[8]-_matrix.data[5]*_matrix.data[7])*inverse;
    data[1] = -(_matrix.data[1]*_matrix.data[8]-_matrix.data[2]*_matrix.data[7])*inverse;
    data[2] = (_matrix.data[1]*_matrix.data[5]-_matrix.data[2]*_matrix.data[4])*inverse;
    data[3] = -(_matrix.data[3]*_matrix.data[8]-_matrix.data[5]*_matrix.data[6])*inverse;
    data[4] = (_matrix.data[0]*_matrix.data[8]-t6)*inverse;
    data[5] = -(t2-t4)*inverse;
    data[6] = (_matrix.data[3]*_matrix.data[7]-_matrix.data[4]*_matrix.data[6])*inverse;
    data[7] = -(_matrix.data[0]*_matrix.data[7]-t5)*inverse;
    data[8] = (t1-t3)*inverse;
}

void IPhysics::Matrix3::SetTranspose(const Matrix3& _matrix){
    data[0] = _matrix.data[0];
    data[1] = _matrix.data[3];
    data[2] = _matrix.data[6];
    data[3] = _matrix.data[1];
    data[4] = _matrix.data[4];
    data[5] = _matrix.data[7];
    data[6] = _matrix.data[2];
    data[7] = _matrix.data[5];
    data[8] = _matrix.data[8];
}

void IPhysics::Matrix3::SetOrientation(const Quaternion& _quaternion){
    data[0] = 1 - (2*_quaternion.j*_quaternion.j + 2*_quaternion.k*_quaternion.k);
    data[1] = 2*_quaternion.i*_quaternion.j + 2*_quaternion.k*_quaternion.r;
    data[2] = 2*_quaternion.i*_quaternion.k - 2*_quaternion.j*_quaternion.r;
    data[3] = 2*_quaternion.i*_quaternion.j - 2*_quaternion.k*_quaternion.r;
    data[4] = 1 - (2*_quaternion.i*_quaternion.i + 2*_quaternion.k*_quaternion.k);
    data[5] = 2*_quaternion.j*_quaternion.k + 2*_quaternion.i*_quaternion.r;
    data[6] = 2*_quaternion.i*_quaternion.k + 2*_quaternion.j*_quaternion.r;
    data[7] = 2*_quaternion.j*_quaternion.k - 2*_quaternion.i*_quaternion.r;
    data[8] = 1 - (2*_quaternion.i*_quaternion.i + 2*_quaternion.j*_quaternion.j);
}

void IPhysics::Matrix3::SetComponents(const Vector3& _componentOne, const Vector3& _componentTwo, const Vector3& _componentThree){
    data[0] = _componentOne.x;
    data[1] = _componentTwo.x;
    data[2] = _componentThree.x;
    data[3] = _componentOne.y;
    data[4] = _componentTwo.y;
    data[5] = _componentThree.y;
    data[6] = _componentOne.z;
    data[7] = _componentTwo.z;
    data[8] = _componentThree.z;
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

IPhysics::Vector3 IPhysics::Matrix3::Transform(const Vector3& _vector3) const{
    return (*this) * _vector3;
}

IPhysics::Vector3 IPhysics::Matrix3::TransformTranspose(const Vector3& _vector3) const{
    return {
        _vector3.x * data[0] + _vector3.y * data[3] + _vector3.z * data[6],
        _vector3.x * data[1] + _vector3.y * data[4] + _vector3.z * data[7],
        _vector3.x * data[2] + _vector3.y * data[5] + _vector3.z * data[8]
    };
}

// Static
IPhysics::Matrix3 IPhysics::Matrix3::LinearInterpolate(const Matrix3& _startMatrix, const Matrix3& _endMatrix, real _proportion){
    Matrix3 result;
    real omp = 1.0f - _proportion;
    for (unsigned i = 0; i < 9; i++){
        result.data[i] = _startMatrix.data[i] * omp + _endMatrix.data[i] * _proportion;
    }
    return result;
}

/*
 * Matrix4
 */

// Constructors
IPhysics::Matrix4::Matrix4() = default;

// Operators
IPhysics::Matrix4 IPhysics::Matrix4::operator*(const Matrix4& _other) const{
    Matrix4 result{};
    result.data[0] = _other.data[0]*data[0] + _other.data[4]*data[1] +
    _other.data[8]*data[2];
    result.data[4] = _other.data[0]*data[4] + _other.data[4]*data[5] +
    _other.data[8]*data[6];
    result.data[8] = _other.data[0]*data[8] + _other.data[4]*data[9] +
    _other.data[8]*data[10];
    result.data[1] = _other.data[1]*data[0] + _other.data[5]*data[1] +
    _other.data[9]*data[2];
    result.data[5] = _other.data[1]*data[4] + _other.data[5]*data[5] +
    _other.data[9]*data[6];
    result.data[9] = _other.data[1]*data[8] + _other.data[5]*data[9] +
    _other.data[9]*data[10];
    result.data[2] = _other.data[2]*data[0] + _other.data[6]*data[1] +
    _other.data[10]*data[2];
    result.data[6] = _other.data[2]*data[4] + _other.data[6]*data[5] +
    _other.data[10]*data[6];
    result.data[10] = _other.data[2]*data[8] + _other.data[6]*data[9] +
    _other.data[10]*data[10];
    result.data[3] = _other.data[3]*data[0] + _other.data[7]*data[1] +
    _other.data[11]*data[2] + data[3];
    result.data[7] = _other.data[3]*data[4] + _other.data[7]*data[5] +
    _other.data[11]*data[6] + data[7];
    result.data[11] = _other.data[3]*data[8] + _other.data[7]*data[9] +
    _other.data[11]*data[10] + data[11];
    return result;
}

IPhysics::Vector3 IPhysics::Matrix4::operator*(const Vector3& _vector3) const{
    return {
        _vector3.x * data[0] + _vector3.y * data[1] + _vector3.z * data[2] + data[3], 
        _vector3.x * data[4] + _vector3.y * data[5] + _vector3.z * data[6] + data[7], 
        _vector3.x * data[8] + _vector3.y * data[9] + _vector3.z * data[10] + data[11]
    };
}

// Mutators
void IPhysics::Matrix4::SetInverse(const Matrix4& _matrix){
    real det = GetDeterminant();
    if (det == 0) return;
    det = static_cast<real>(1.0f)/det;
    data[0] = (-_matrix.data[9]*_matrix.data[6]+_matrix.data[5]*_matrix.data[10])*det;
    data[4] = (_matrix.data[8]*_matrix.data[6]-_matrix.data[4]*_matrix.data[10])*det;
    data[8] = (-_matrix.data[8]*_matrix.data[5]+_matrix.data[4]*_matrix.data[9]*_matrix.data[15])*det;
    data[1] = (_matrix.data[9]*_matrix.data[2]-_matrix.data[1]*_matrix.data[10])*det;
    data[5] = (-_matrix.data[8]*_matrix.data[2]+_matrix.data[0]*_matrix.data[10])*det;
    data[9] = (_matrix.data[8]*_matrix.data[1]-_matrix.data[0]*_matrix.data[9]*_matrix.data[15])*det;
    data[2] = (-_matrix.data[5]*_matrix.data[2]+_matrix.data[1]*_matrix.data[6]*_matrix.data[15])*det;
    data[6] = (+_matrix.data[4]*_matrix.data[2]-_matrix.data[0]*_matrix.data[6]*_matrix.data[15])*det;
    data[10] = (-_matrix.data[4]*_matrix.data[1]+_matrix.data[0]*_matrix.data[5]*_matrix.data[15])*det;
    data[3] = (_matrix.data[9]*_matrix.data[6]*_matrix.data[3]
    -_matrix.data[5]*_matrix.data[10]*_matrix.data[3]
    -_matrix.data[9]*_matrix.data[2]*_matrix.data[7]
    +_matrix.data[1]*_matrix.data[10]*_matrix.data[7]
    +_matrix.data[5]*_matrix.data[2]*_matrix.data[11]
    -_matrix.data[1]*_matrix.data[6]*_matrix.data[11])*det;
    data[7] = (-_matrix.data[8]*_matrix.data[6]*_matrix.data[3]
    +_matrix.data[4]*_matrix.data[10]*_matrix.data[3]
    +_matrix.data[8]*_matrix.data[2]*_matrix.data[7]
    -_matrix.data[0]*_matrix.data[10]*_matrix.data[7]
    -_matrix.data[4]*_matrix.data[2]*_matrix.data[11]
    +_matrix.data[0]*_matrix.data[6]*_matrix.data[11])*det;
    data[11] =(_matrix.data[8]*_matrix.data[5]*_matrix.data[3]
    -_matrix.data[4]*_matrix.data[9]*_matrix.data[3]
    -_matrix.data[8]*_matrix.data[1]*_matrix.data[7]
    +_matrix.data[0]*_matrix.data[9]*_matrix.data[7]
    +_matrix.data[4]*_matrix.data[1]*_matrix.data[11]
    -_matrix.data[0]*_matrix.data[5]*_matrix.data[11])*det;
}

void IPhysics::Matrix4::SetOrientationAndPos(const Quaternion& _quaternion, const Vector3& _position){
    data[0] = 1 - (2*_quaternion.j*_quaternion.j + 2*_quaternion.k*_quaternion.k);
    data[1] = 2*_quaternion.i*_quaternion.j + 2*_quaternion.k*_quaternion.r;
    data[2] = 2*_quaternion.i*_quaternion.k - 2*_quaternion.j*_quaternion.r;
    data[3] = _position.x;
    data[4] = 2*_quaternion.i*_quaternion.j - 2*_quaternion.k*_quaternion.r;
    data[5] = 1 - (2*_quaternion.i*_quaternion.i + 2*_quaternion.k*_quaternion.k);
    data[6] = 2*_quaternion.j*_quaternion.k + 2*_quaternion.i*_quaternion.r;
    data[7] = _position.y;
    data[8] = 2*_quaternion.i*_quaternion.k + 2*_quaternion.j*_quaternion.r;
    data[9] = 2*_quaternion.j*_quaternion.k - 2*_quaternion.i*_quaternion.r;
    data[10] = 1 - (2*_quaternion.i*_quaternion.i + 2*_quaternion.j*_quaternion.j);
    data[11] = _position.z;
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

IPhysics::Vector3 IPhysics::Matrix4::Transform(const Vector3& _vector3) const{
    return (*this) * _vector3;
}

IPhysics::Vector3 IPhysics::Matrix4::TransformInverse(const Vector3& _vector3) const{
    Vector3 tmp = _vector3;
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

IPhysics::Vector3 IPhysics::Matrix4::TransformDirection(const Vector3& _vector3) const{
    return {
        _vector3.x * data[0] +
        _vector3.y * data[1] +
        _vector3.z * data[2],
        _vector3.x * data[4] +
        _vector3.y * data[5] +
        _vector3.z * data[6],
        _vector3.x * data[8] +
        _vector3.y * data[9] +
        _vector3.z * data[10]
    };
}

IPhysics::Vector3 IPhysics::Matrix4::TransformInverseDirection(const Vector3& _vector3) const{
    return {
        _vector3.x * data[0] +
        _vector3.y * data[4] +
        _vector3.z * data[8],
        _vector3.x * data[1] +
        _vector3.y * data[5] +
        _vector3.z * data[9],
        _vector3.x * data[2] +
        _vector3.y * data[6] +
        _vector3.z * data[10]
    };
}

IPhysics::Vector3 IPhysics::Matrix4::GetAxisVector(int _index) const{
    return {data[_index], data[_index + 4], data[_index + 8]};
}

/*
 * Random number generator
 */
IPhysics::real IPhysics::RandomStore::RandomReal(real _lowerBound, real _upperbound){
    std::uniform_real_distribution<real> doubleDistribution(_lowerBound, _upperbound);
    return doubleDistribution(RandomStore::generator);
}

int IPhysics::RandomStore::RandomInt(int _lowerBound, int _upperbound){
    std::uniform_int_distribution<int> intDistribution(_lowerBound, _upperbound);
    return intDistribution(RandomStore::generator);
}

IPhysics::Vector3 IPhysics::RandomStore::RandomVector3(const Vector3 &_lowerBound, const Vector3 &_upperbound){
    return {RandomReal(_lowerBound.x, _upperbound.x), RandomReal(_lowerBound.y, _upperbound.y), RandomReal(_lowerBound.z, _upperbound.z)};
}

/*
 * Local and world transforms
 */
IPhysics::Vector3 IPhysics::LocalWorldTransforms::LocalToWorld(const Vector3& _local, const Matrix4& _transform){
    return _transform.Transform(_local);
}

IPhysics::Vector3 IPhysics::LocalWorldTransforms::WorldToLocal(const Vector3& _world, const Matrix4& _transform){
    return _transform.TransformInverse(_world);
}

IPhysics::Vector3 IPhysics::LocalWorldTransforms::LocalToWorldDirection(const Vector3& _local, const Matrix4& _transform){
    return _transform.TransformDirection(_local);
}

IPhysics::Vector3 IPhysics::LocalWorldTransforms::WorldToLocalDirection(const Vector3& _world, const Matrix4& _transform){
    return _transform.TransformInverseDirection(_world);
}

/*
 *  Mathematical operations on real.
 */
IPhysics::real IPhysics::RealSqrt(real _value){
    return std::sqrt(_value);
}

IPhysics::real IPhysics::RealPow(real _value, real _power){
    return std::pow(_value, _power);
}

IPhysics::real IPhysics::RealAbs(real _value){
    return std::abs(_value);
}
