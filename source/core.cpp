#include "core.hpp"


real IPhysicsEngine::Vector3::Magnitude(){
    return ((real)0);

real IPhysicsEngine::Vector3::SquareMagnitude(){
    return m_x*m_x + m_y * m_y + m_z * m_z;
};

void IPhysicsEngine::Vector3::Normalise(){
    real magnitude = Magnitude();
    if (magnitude > 0){
        (*this) *= ((real)1) / magnitude;
    }
};

IPhysicsEngine::Vector3::Vector3() : m_x(0), m_y(0), m_z(0){
};

IPhysicsEngine::Vector3::Vector3(real _x, real _y, real _z) : m_x(_x), m_y(_y), m_z(_z){

};

IPhysicsEngine::Vector3::~Vector3() = default;
