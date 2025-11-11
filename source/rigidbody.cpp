#include "rigidbody.hpp"

IPhysicsEngine::RigidBody::RigidBody(const Vector3& _originalPosition, const Quaternion& _originalOrientation, const real& _inverseMass, const real& _linearDamping, const real& _angularDamping, const Matrix3& _inverseInertiaTensor) : m_position(_originalPosition), m_orientation(_originalOrientation), m_inverseMass(_inverseMass), m_linearDamping(_linearDamping), m_inverseInertiaTensor(_inverseInertiaTensor){

}

void IPhysicsEngine::RigidBody::Integrate(real _duration){

    // Linear Acceleration
    m_lastFrameAcceleration = m_acceleration;
    m_lastFrameAcceleration.AddScaledVector(m_forceAccumulated, m_inverseMass);

    // Angular Acceleration
    Vector3 AngularAcceleration = m_inverseInertiaTensorWorld.Transform(m_torqueAccumulated);

    m_velocity.AddScaledVector(m_lastFrameAcceleration, _duration);

    m_rotation.AddScaledVector(AngularAcceleration, _duration);

    m_velocity *= RealPow(m_linearDamping, _duration);
    m_rotation *= RealPow(m_angularDamping, _duration);

    m_position.AddScaledVector(m_velocity, _duration);

    m_orientation.AddScaledVector(m_rotation, _duration);

    CalculateDerivedData();

    ClearAccumulators();
}

void IPhysicsEngine::RigidBody::AddForce(const Vector3& _vector){
    m_forceAccumulated += _vector;
    m_isAwake = true;
}


void IPhysicsEngine::RigidBody::AddForceAtPoint(const Vector3& _vector, const Vector3& _point){
    // This converts to coordinates relative to the centre of mass.
    Vector3 pt = _point;
    pt -= m_position;

    m_forceAccumulated += _vector;
    m_torqueAccumulated += pt % _vector;
    m_isAwake = true;
}

void IPhysicsEngine::RigidBody::AddForceAtBodyPoint(const Vector3& _vector, const Vector3& _point){
    Vector3 pt = GetPointInWorldSpace(_point);
    AddForceAtPoint(_vector, pt);

    m_isAwake = true;
}

void IPhysicsEngine::RigidBody::ClearAccumulators(){
    m_forceAccumulated.Clear();
    m_torqueAccumulated.Clear();
}

void IPhysicsEngine::RigidBody::SetInertiaTensor(const Matrix3& _inertiaTensor){
    m_inverseInertiaTensor.SetInverse(_inertiaTensor);
}

void IPhysicsEngine::RigidBody::CalculateDerivedData(){
    m_orientation.Normalise();

    CalculateTransformMatrix(m_transformMatrix, m_position, m_orientation);
    CalculateTransformInertiaTensor(m_inverseInertiaTensorWorld, m_orientation, m_inverseInertiaTensor, m_transformMatrix);
}

IPhysicsEngine::Vector3 IPhysicsEngine::RigidBody::GetPointInLocalSpace(const Vector3& _point){
    return m_transformMatrix.TransformInverse(_point);
}

IPhysicsEngine::Vector3 IPhysicsEngine::RigidBody::GetPointInWorldSpace(const Vector3& _point){
    return m_transformMatrix.Transform(_point);
}

IPhysicsEngine::real IPhysicsEngine::RigidBody::GetMass(){
    return 1 / m_inverseMass;
}

IPhysicsEngine::Vector3& IPhysicsEngine::RigidBody::GetPosition(){
    return m_position;
}

bool IPhysicsEngine::RigidBody::HasFiniteMass(){
    return m_inverseMass >= 0.0f;
}

void IPhysicsEngine::RigidBody::CalculateTransformMatrix(Matrix4& _transformMatrix, const Vector3& _position, const Quaternion& _orientation){
    _transformMatrix.data[0] = 1-2*_orientation.j*_orientation.j -
    2*_orientation.k*_orientation.k;
    _transformMatrix.data[1] = 2*_orientation.i*_orientation.j -
    2*_orientation.r*_orientation.k;
    _transformMatrix.data[2] = 2*_orientation.i*_orientation.k +
    2*_orientation.r*_orientation.j;
    _transformMatrix.data[3] = _position.GetX();
    _transformMatrix.data[4] = 2*_orientation.i*_orientation.j +
    2*_orientation.r*_orientation.k;
    _transformMatrix.data[5] = 1-2*_orientation.i*_orientation.i -
    2*_orientation.k*_orientation.k;
    _transformMatrix.data[6] = 2*_orientation.j*_orientation.k -
    2*_orientation.r*_orientation.i;
    _transformMatrix.data[7] = _position.GetY();
    _transformMatrix.data[8] = 2*_orientation.i*_orientation.k -
    2*_orientation.r*_orientation.j;
    _transformMatrix.data[9] = 2*_orientation.j*_orientation.k +
    2*_orientation.r*_orientation.i;
    _transformMatrix.data[10] = 1-2*_orientation.i*_orientation.i -
    2*_orientation.j*_orientation.j;
    _transformMatrix.data[11] = _position.GetZ();
}

void IPhysicsEngine::RigidBody::CalculateTransformInertiaTensor(Matrix3& _iitWorld, const Quaternion& _quaternion, const Matrix3& _iitBody, const Matrix4& _rotmat){
    real t4 = _rotmat.data[0]*_iitBody.data[0]+
    _rotmat.data[1]*_iitBody.data[3]+
    _rotmat.data[2]*_iitBody.data[6];
    real t9 = _rotmat.data[0]*_iitBody.data[1]+
    _rotmat.data[1]*_iitBody.data[4]+
    _rotmat.data[2]*_iitBody.data[7];
    real t14 = _rotmat.data[0]*_iitBody.data[2]+
    _rotmat.data[1]*_iitBody.data[5]+
    _rotmat.data[2]*_iitBody.data[8];
    real t28 = _rotmat.data[4]*_iitBody.data[0]+
    _rotmat.data[5]*_iitBody.data[3]+
    _rotmat.data[6]*_iitBody.data[6];
    real t33 = _rotmat.data[4]*_iitBody.data[1]+
    _rotmat.data[5]*_iitBody.data[4]+
    _rotmat.data[6]*_iitBody.data[7];
    real t38 = _rotmat.data[4]*_iitBody.data[2]+
    _rotmat.data[5]*_iitBody.data[5]+
    _rotmat.data[6]*_iitBody.data[8];
    real t52 = _rotmat.data[8]*_iitBody.data[0]+
    _rotmat.data[9]*_iitBody.data[3]+
    _rotmat.data[10]*_iitBody.data[6];
    real t57 = _rotmat.data[8]*_iitBody.data[1]+
    _rotmat.data[9]*_iitBody.data[4]+
    _rotmat.data[10]*_iitBody.data[7];
    real t62 = _rotmat.data[8]*_iitBody.data[2]+
    _rotmat.data[9]*_iitBody.data[5]+
    _rotmat.data[10]*_iitBody.data[8];
    _iitWorld.data[0] = t4*_rotmat.data[0]+
    t9*_rotmat.data[1]+
    t14*_rotmat.data[2];
    _iitWorld.data[1] = t4*_rotmat.data[4]+
    t9*_rotmat.data[5]+
    t14*_rotmat.data[6];
    _iitWorld.data[2] = t4*_rotmat.data[8]+
    t9*_rotmat.data[9]+
    t14*_rotmat.data[10];
    _iitWorld.data[3] = t28*_rotmat.data[0]+
    t33*_rotmat.data[1]+
    t38*_rotmat.data[2];
    _iitWorld.data[4] = t28*_rotmat.data[4]+
    t33*_rotmat.data[5]+
    t38*_rotmat.data[6];
    _iitWorld.data[5] = t28*_rotmat.data[8]+
    t33*_rotmat.data[9]+
    t38*_rotmat.data[10];
    _iitWorld.data[6] = t52*_rotmat.data[0]+
    t57*_rotmat.data[1]+
    t62*_rotmat.data[2];
    _iitWorld.data[7] = t52*_rotmat.data[4]+
    t57*_rotmat.data[5]+
    t62*_rotmat.data[6];
    _iitWorld.data[8] = t52*_rotmat.data[8]+
    t57*_rotmat.data[9]+
    t62*_rotmat.data[10];
}
