#include "core.hpp"
#include "precision.hpp"

IPhysicsEngine::real IPhysicsEngine::Vector3::Magnitude(){
    return IPhysicsEngine::RealSqrt(m_x*m_x+m_y*m_y+m_z*m_z);
}

IPhysicsEngine::real IPhysicsEngine::Vector3::SquareMagnitude(){
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
    return m_x * _vector.m_x + m_y * _vector.m_y + _vector.m_z;
}

IPhysicsEngine::Vector3::Vector3() : m_x(0), m_y(0), m_z(0){
};

IPhysicsEngine::Vector3::Vector3(real _x, real _y, real _z) : m_x(_x), m_y(_y), m_z(_z){

};

IPhysicsEngine::Vector3::~Vector3() = default;

IPhysicsEngine::real IPhysicsEngine::Vector3::GetX(){
    return m_x;
}

IPhysicsEngine::real IPhysicsEngine::Vector3::GetY(){
    return m_y;
}

IPhysicsEngine::real IPhysicsEngine::Vector3::GetZ(){
    return m_z;
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::ComponentProduct(const Vector3& _vector){
    return Vector3(m_x * _vector.m_x, m_y * _vector.m_y, m_z * _vector.m_z);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::VectorProduct(const Vector3& _vector){
    return Vector3(m_y * _vector.m_z - m_z * _vector.m_y, m_z * _vector.m_x - m_x * _vector.m_z, m_x * _vector.m_y - m_y * _vector.m_x);
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

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::operator%(const Vector3& _vector){
    return Vector3(m_y * _vector.m_z - m_z * _vector.m_y, m_z * _vector.m_x - m_x * _vector.m_z, m_x * _vector.m_y - m_y * _vector.m_x);
}

IPhysicsEngine::real IPhysicsEngine::RealSqrt(real _value){
    return sqrt(_value);
}

IPhysicsEngine::real IPhysicsEngine::RealPow(real _value, real _power){
    return pow(_value, _power);
}