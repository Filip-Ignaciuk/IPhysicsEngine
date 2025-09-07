#include "core.hpp"
#include "precision.hpp"

IPhysicsEngine::real IPhysicsEngine::Vector3::Magnitude(){
    return real_sqrt(x*x+y*y+z*z);
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

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::ComponentProduct(const Vector3& _vector){
    return Vector3(m_x * _vector.m_x, m_y * _vector.m_y, m_z * _vector.m_z);
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

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::operator+(const Vector3& _vector){
    return Vector3(m_x + _vector.m_x, m_y + _vector.m_y, m_z + _vector.m_z);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Vector3::operator-(const Vector3& _vector){
    return Vector3(m_x - _vector.m_x, m_y - _vector.m_y, m_z - _vector.m_z);
}

IPhysicsEngine::real IPhysicsEngine::Vector3::operator*(const Vector3& _vector){
    return m_x * _vector.m_x + m_y * _vector.m_y + m_z * _vector.m_z;
}

