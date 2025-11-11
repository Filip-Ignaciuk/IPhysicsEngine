#include "core.hpp"
#include "precision.hpp"

IPhysicsEngine::real IPhysicsEngine::Vector3::Magnitude() const{
    return IPhysicsEngine::RealSqrt(m_x*m_x+m_y*m_y+m_z*m_z);
}

IPhysicsEngine::real IPhysicsEngine::Vector3::SquareMagnitude() const{
    return m_x*m_x + m_y * m_y + m_z * m_z;
};

void IPhysicsEngine::Vector3::Normalise(){
    real magnitude = Magnitude();
    if (magnitude > 0){
        (*this) *= ((real)1) / magnitude;
    }
};

void IPhysicsEngine::Vector3::AddScaledVector(const Vector3& _vector,  real scale){
    m_x += _vector.m_x * scale;
    m_y += _vector.m_y * scale;
    m_z += _vector.m_z * scale;
};

void IPhysicsEngine::Vector3::ComponentProductUpdate(const Vector3& _vector){
    m_x = _vector.m_x;
    m_y = _vector.m_y;
    m_z = _vector.m_z;
};

IPhysicsEngine::real IPhysicsEngine::Vector3::ScalarProduct(const Vector3& _vector){
    return m_x * _vector.m_x + m_y * _vector.m_y + m_z * _vector.m_z;
}

IPhysicsEngine::Vector3::Vector3() : m_x(0), m_y(0), m_z(0){
};

IPhysicsEngine::Vector3::Vector3(real _x, real _y, real _z) : m_x(_x), m_y(_y), m_z(_z){

};

IPhysicsEngine::Vector3::~Vector3() = default;

IPhysicsEngine::real IPhysicsEngine::Vector3::GetX() const{
    return m_x;
}

IPhysicsEngine::real IPhysicsEngine::Vector3::GetY() const{
    return m_y;
}

IPhysicsEngine::real IPhysicsEngine::Vector3::GetZ() const{
    return m_z;
}

void IPhysicsEngine::Vector3::SetX(real _x){
    m_x = _x;
}

void IPhysicsEngine::Vector3::SetY(real _y){
    m_y = _y;
}

void IPhysicsEngine::Vector3::SetZ(real _z){
    m_z = _z;
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::ComponentProduct(const Vector3& _vector){
    return Vector3(m_x * _vector.m_x, m_y * _vector.m_y, m_z * _vector.m_z);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::VectorProduct(const Vector3& _vector){
    return Vector3(m_y * _vector.m_z - m_z * _vector.m_y, m_z * _vector.m_x - m_x * _vector.m_z, m_x * _vector.m_y - m_y * _vector.m_x);
}

void IPhysicsEngine::Vector3::Clear(){
    m_x = 0;
    m_y = 0;
    m_z = 0;
}

void IPhysicsEngine::Vector3::MakeOrthonormalBasis(Vector3* _vectorA, Vector3* _vectorB, Vector3* _vectorC){
    _vectorA->Normalise();
    *_vectorC = (*_vectorA) % (*_vectorB);
    if (_vectorC->SquareMagnitude() == 0.0){
        return;
    }
    _vectorC->Normalise();
    *_vectorB = (*_vectorC) % (*_vectorA);
}

void IPhysicsEngine::Vector3::operator*=(const real _value){
    m_x *= _value;
    m_y *= _value;
    m_z *= _value;
}

void IPhysicsEngine::Vector3::operator+=(const Vector3& _vector){
    m_x += _vector.m_x;
    m_y += _vector.m_y;
    m_z += _vector.m_z;
}

void IPhysicsEngine::Vector3::operator-=(const Vector3& _vector){
    m_x -= _vector.m_x;
    m_y -= _vector.m_y;
    m_z -= _vector.m_z;
}

void IPhysicsEngine::Vector3::operator%=(const Vector3& _vector){
    *this = VectorProduct(_vector);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::operator+(const Vector3& _vector){
    return Vector3(m_x + _vector.m_x, m_y + _vector.m_y, m_z + _vector.m_z);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::operator-(const Vector3& _vector){
    return Vector3(m_x - _vector.m_x, m_y - _vector.m_y, m_z - _vector.m_z);
}

IPhysicsEngine::real IPhysicsEngine::Vector3::operator*(const Vector3& _vector){
    return m_x * _vector.m_x + m_y * _vector.m_y + m_z * _vector.m_z;
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::operator*(const IPhysicsEngine::real& _magnitude){
    return Vector3(m_x * _magnitude, m_y * _magnitude, m_z * _magnitude);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::operator%(const Vector3& _vector){
    return Vector3(m_y * _vector.m_z - m_z * _vector.m_y, m_z * _vector.m_x - m_x * _vector.m_z, m_x * _vector.m_y - m_y * _vector.m_x);
}

IPhysicsEngine::Quaternion::Quaternion() : r(0.0f), i(0.0f), j(0.0f), k(0.0f){
}

IPhysicsEngine::Quaternion::Quaternion(real _r, real _i, real _j, real _k) : r(_r), i(_i), j(_j), k(_k){

}

void IPhysicsEngine::Quaternion::Normalise(){
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

void IPhysicsEngine::Quaternion::operator*= (const Quaternion& _mulitplier){
    Quaternion quaternion = *this;
    r = quaternion.r*_mulitplier.r - quaternion.i*_mulitplier.i -
    quaternion.j*_mulitplier.j - quaternion.k*_mulitplier.k;
    i = quaternion.r*_mulitplier.i + quaternion.i*_mulitplier.r +
    quaternion.j*_mulitplier.k - quaternion.k*_mulitplier.j;
    j = quaternion.r*_mulitplier.j + quaternion.j*_mulitplier.r +
    quaternion.k*_mulitplier.i - quaternion.i*_mulitplier.k;
    k = quaternion.r*_mulitplier.k + quaternion.k*_mulitplier.r +
    quaternion.i*_mulitplier.j - quaternion.j*_mulitplier.i;
}

void IPhysicsEngine::Quaternion::RotateByVector(const Vector3& _vector3){
    Quaternion q(0, _vector3.GetX(), _vector3.GetY(), _vector3.GetZ());
    (*this) *= q;
}

void IPhysicsEngine::Quaternion::AddScaledVector(const Vector3& _vector3, real _scale){
    Quaternion q(0,
    _vector3.GetX() * _scale,
    _vector3.GetY() * _scale,
    _vector3.GetZ() * _scale);
    q *= *this;
    r += q.r * ((real)0.5);
    i += q.i * ((real)0.5);
    j += q.j * ((real)0.5);
    k += q.k * ((real)0.5);
}

IPhysicsEngine::Matrix3::Matrix3(){
    for (size_t i = 0; i < 9; i++)
    {
        data[i] = 0.0f;
    }
    
}

IPhysicsEngine::Matrix3::Matrix3(real _a1, real _a2, real _a3, real _b1, real _b2, real _b3, real _c1, real _c2, real _c3){
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

IPhysicsEngine::Matrix3 IPhysicsEngine::Matrix3::operator*(const Matrix3& _other) const{
    return Matrix3(
        data[0]*_other.data[0] + data[1]*_other.data[3] + data[2]*_other.data[6],
        data[0]*_other.data[1] + data[1]*_other.data[4] + data[2]*_other.data[7],
        data[0]*_other.data[2] + data[1]*_other.data[5] + data[2]*_other.data[8],
        data[3]*_other.data[0] + data[4]*_other.data[3] + data[5]*_other.data[6],
        data[3]*_other.data[1] + data[4]*_other.data[4] + data[5]*_other.data[7],
        data[3]*_other.data[2] + data[4]*_other.data[5] + data[5]*_other.data[8],
        data[6]*_other.data[0] + data[7]*_other.data[3] + data[8]*_other.data[6],
        data[6]*_other.data[1] + data[7]*_other.data[4] + data[8]*_other.data[7],
        data[6]*_other.data[2] + data[7]*_other.data[5] + data[8]*_other.data[8]
    );
}

void IPhysicsEngine::Matrix3::operator*=(const Matrix3& _other){
    real t1;
    real t2;
    real t3;
    t1 = data[0]*_other.data[0] + data[1]*_other.data[3] + data[2]*_other.data[6];
    t2 = data[0]*_other.data[1] + data[1]*_other.data[4] + data[2]*_other.data[7];
    t3 = data[0]*_other.data[2] + data[1]*_other.data[5] + data[2]*_other.data[8];
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

void IPhysicsEngine::Matrix3::SetInverse(const Matrix3& _matrix){
    real t1 = _matrix.data[0]*_matrix.data[4];
    real t2 = _matrix.data[0]*_matrix.data[5];
    real t3 = _matrix.data[1]*_matrix.data[3];
    real t4 = _matrix.data[2]*_matrix.data[3];
    real t5 = _matrix.data[1]*_matrix.data[6];
    real t6 = _matrix.data[2]*_matrix.data[6];
    real det = (t1*_matrix.data[8] - t2*_matrix.data[7] - t3*_matrix.data[8]+
    t4*_matrix.data[7] + t5*_matrix.data[5] - t6*_matrix.data[4]);
    if (det == (real)0.0f) return;
    real invd = (real)1.0f/det;

    data[0] = (_matrix.data[4]*_matrix.data[8]-_matrix.data[5]*_matrix.data[7])*invd;
    data[1] = -(_matrix.data[1]*_matrix.data[8]-_matrix.data[2]*_matrix.data[7])*invd;
    data[2] = (_matrix.data[1]*_matrix.data[5]-_matrix.data[2]*_matrix.data[4])*invd;
    data[3] = -(_matrix.data[3]*_matrix.data[8]-_matrix.data[5]*_matrix.data[6])*invd;
    data[4] = (_matrix.data[0]*_matrix.data[8]-t6)*invd;
    data[5] = -(t2-t4)*invd;
    data[6] = (_matrix.data[3]*_matrix.data[7]-_matrix.data[4]*_matrix.data[6])*invd;
    data[7] = -(_matrix.data[0]*_matrix.data[7]-t5)*invd;
    data[8] = (t1-t3)*invd;
}

void IPhysicsEngine::Matrix3::SetTranspose(const Matrix3& _matrix){
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

void IPhysicsEngine::Matrix3::SetOrientation(const Quaternion& _quaternion){
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

IPhysicsEngine::Matrix3 IPhysicsEngine::Matrix3::Transpose() const{
    Matrix3 result;
    result.SetTranspose(*this);
    return result;
}

IPhysicsEngine::Matrix3 IPhysicsEngine::Matrix3::Inverse() const{
    Matrix3 result;
    result.SetInverse(*this);
    return result;
}

void IPhysicsEngine::Matrix3::Invert(){
    SetInverse(*this);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Matrix3::operator*(const Vector3& _vector3) const{
    return Vector3(
        _vector3.GetX() * data[0] + _vector3.GetY() * data[1] + _vector3.GetZ() * data[2], 
        _vector3.GetX() * data[3] + _vector3.GetY() * data[4] + _vector3.GetZ() * data[5], 
        _vector3.GetX() * data[6] + _vector3.GetY() * data[7] + _vector3.GetZ() * data[8]
    );
}

IPhysicsEngine::Vector3 IPhysicsEngine::Matrix3::Transform(const Vector3& _vector3) const{
    return (*this) * _vector3;
}

IPhysicsEngine::Matrix4 IPhysicsEngine::Matrix4::operator*(const Matrix4& _other) const{
    Matrix4 result;
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

IPhysicsEngine::Vector3 IPhysicsEngine::Matrix4::operator*(const Vector3& _vector3) const{
    return Vector3(
        _vector3.GetX() * data[0] + _vector3.GetY() * data[1] + _vector3.GetZ() * data[2] + data[3], 
        _vector3.GetX() * data[4] + _vector3.GetY() * data[5] + _vector3.GetZ() * data[6] + data[7], 
        _vector3.GetX() * data[8] + _vector3.GetY() * data[9] + _vector3.GetZ() * data[10] + data[11]
    );
}

void IPhysicsEngine::Matrix4::SetInverse(const Matrix4& _matrix){
    real det = GetDeterminant();
    if (det == 0) return;
    det = ((real)1.0f)/det;
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

void IPhysicsEngine::Matrix4::SetOrientationAndPos(const Quaternion& _quaternion, const Vector3& _position){
    data[0] = 1 - (2*_quaternion.j*_quaternion.j + 2*_quaternion.k*_quaternion.k);
    data[1] = 2*_quaternion.i*_quaternion.j + 2*_quaternion.k*_quaternion.r;
    data[2] = 2*_quaternion.i*_quaternion.k - 2*_quaternion.j*_quaternion.r;
    data[3] = _position.GetX();
    data[4] = 2*_quaternion.i*_quaternion.j - 2*_quaternion.k*_quaternion.r;
    data[5] = 1 - (2*_quaternion.i*_quaternion.i + 2*_quaternion.k*_quaternion.k);
    data[6] = 2*_quaternion.j*_quaternion.k + 2*_quaternion.i*_quaternion.r;
    data[7] = _position.GetY();
    data[8] = 2*_quaternion.i*_quaternion.k + 2*_quaternion.j*_quaternion.r;
    data[9] = 2*_quaternion.j*_quaternion.k - 2*_quaternion.i*_quaternion.r;
    data[10] = 1 - (2*_quaternion.i*_quaternion.i + 2*_quaternion.j*_quaternion.j);
    data[11] = _position.GetZ();
}

IPhysicsEngine::Vector3 IPhysicsEngine::Matrix4::Transform(const Vector3& _vector3) const{
    return (*this) * _vector3;
}

IPhysicsEngine::real IPhysicsEngine::Matrix4::GetDeterminant() const{
    return  data[8]*data[5]*data[2]+
            data[4]*data[9]*data[2]+
            data[8]*data[1]*data[6]-
            data[0]*data[9]*data[6]-
            data[4]*data[1]*data[10]+
            data[0]*data[5]*data[10];
}

IPhysicsEngine::Matrix4 IPhysicsEngine::Matrix4::Inverse() const{
    Matrix4 result;
    result.SetInverse(*this);
    return result;
}

void IPhysicsEngine::Matrix4::Invert(){
    SetInverse(*this);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Matrix4::TransformInverse(const Vector3& _vector3) const{
    Vector3 tmp = _vector3;
    real x = tmp.GetX();
    real y = tmp.GetY();
    real z = tmp.GetZ();
    tmp.SetX(x -= data[3]);
    tmp.SetY(y -= data[7]);
    tmp.SetZ(z -= data[11]);
    return Vector3(
        tmp.GetX() * data[0] +
        tmp.GetY() * data[4] +
        tmp.GetZ() * data[8],
        tmp.GetX() * data[1] +
        tmp.GetY() * data[5] +
        tmp.GetZ() * data[9],
        tmp.GetX() * data[2] +
        tmp.GetY() * data[6] +
        tmp.GetZ() * data[10]
    );
}

IPhysicsEngine::Vector3 IPhysicsEngine::Matrix4::TransformDirection(const Vector3& _vector3) const{
    return Vector3(
        _vector3.GetX() * data[0] +
        _vector3.GetY() * data[1] +
        _vector3.GetZ() * data[2],
        _vector3.GetX() * data[4] +
        _vector3.GetY() * data[5] +
        _vector3.GetZ() * data[6],
        _vector3.GetX() * data[8] +
        _vector3.GetY() * data[9] +
        _vector3.GetZ() * data[10]
    );
}

IPhysicsEngine::Vector3 IPhysicsEngine::Matrix4::TransformInverseDirection(const Vector3& _vector3) const{
    return Vector3(
        _vector3.GetX() * data[0] +
        _vector3.GetY() * data[4] +
        _vector3.GetZ() * data[8],
        _vector3.GetX() * data[1] +
        _vector3.GetY() * data[5] +
        _vector3.GetZ() * data[9],
        _vector3.GetX() * data[2] +
        _vector3.GetY() * data[6] +
        _vector3.GetZ() * data[10]
    );
}


IPhysicsEngine::real IPhysicsEngine::RealSqrt(real _value){
    return sqrt(_value);
}

IPhysicsEngine::real IPhysicsEngine::RealPow(real _value, real _power){
    return pow(_value, _power);
}

IPhysicsEngine::real IPhysicsEngine::RealAbs(real _value){
    return abs(_value);
}

void IPhysicsEngine::RandomStore::Initialise(){
    std::mt19937 newGenerator(randomDevice());
    generator = newGenerator;
}

IPhysicsEngine::Vector3 IPhysicsEngine::RandomVector3(Vector3 _lowerBound, Vector3 _upperbound){
    return Vector3(RandomReal(_lowerBound.GetX(), _upperbound.GetX()), RandomReal(_lowerBound.GetY(), _upperbound.GetY()), RandomReal(_lowerBound.GetZ(), _upperbound.GetZ()));
}

IPhysicsEngine::real IPhysicsEngine::RandomReal(real _lowerBound, real _upperbound){
    

    std::uniform_real_distribution<double> doubleDistribution(_lowerBound, _upperbound);

    return doubleDistribution(RandomStore::generator);

}

int IPhysicsEngine::RandomInt(int _lowerBound, int _upperbound){

    std::uniform_real_distribution<double> intDistribution(_lowerBound, _upperbound);

    return intDistribution(RandomStore::generator);
}
