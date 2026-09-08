#include "rigidbody.hpp"

/*
 * RigidBody
 */

// Constructors
IPhysics::RigidBody::RigidBody() = default;

IPhysics::RigidBody::RigidBody(const Vector3& originalPosition,
                               const Quaternion& originalOrientation,
                               const real& inverseMass,
                               const real& linearDamping,
                               const real& angularDamping,
                               const Matrix3& inverseInertiaTensor)
    : m_position(originalPosition),
      m_orientation(originalOrientation),
      m_inverseMass(inverseMass),
      m_linearDamping(linearDamping),
      m_inverseInertiaTensor(inverseInertiaTensor) {}

// Mutators
void IPhysics::RigidBody::Integrate(real duration) {
  // Linear Acceleration
  m_lastFrameAcceleration = m_acceleration;
  m_lastFrameAcceleration.AddScaledVector(m_forceAccumulated, m_inverseMass);

  // Angular Acceleration
  const Vector3 AngularAcceleration =
      m_inverseInertiaTensorWorld.Transform(m_torqueAccumulated);

  m_velocity.AddScaledVector(m_lastFrameAcceleration, duration);

  m_rotation.AddScaledVector(AngularAcceleration, duration);

  m_velocity *= RealPow(m_linearDamping, duration);
  m_rotation *= RealPow(m_angularDamping, duration);

  m_position.AddScaledVector(m_velocity, duration);

  m_orientation.AddScaledVector(m_rotation, duration);

  CalculateDerivedData();

  ClearAccumulators();
}

void IPhysics::RigidBody::AddForce(const Vector3& vector) {
  m_forceAccumulated += vector;
  m_isAwake = true;
}

void IPhysics::RigidBody::AddForceAtPoint(const Vector3& vector,
                                          const Vector3& point) {
  // This converts to coordinates relative to the centre of mass.
  Vector3 pt = point;
  pt -= m_position;

  m_forceAccumulated += vector;
  m_torqueAccumulated += pt % vector;
  m_isAwake = true;
}

void IPhysics::RigidBody::AddForceAtBodyPoint(const Vector3& vector,
                                              const Vector3& point) {
  const Vector3 pt = GetPointInWorldSpace(point);
  AddForceAtPoint(vector, pt);

  m_isAwake = true;
}

void IPhysics::RigidBody::ClearAccumulators() {
  m_forceAccumulated.Clear();
  m_torqueAccumulated.Clear();
}

void IPhysics::RigidBody::CalculateDerivedData() {
  m_orientation.Normalise();

  CalculateTransformMatrix(m_transformMatrix, m_position, m_orientation);
  CalculateTransformInertiaTensor(m_inverseInertiaTensorWorld, m_orientation,
                                  m_inverseInertiaTensor, m_transformMatrix);
}

void IPhysics::RigidBody::SetPosition(const Vector3& position) {
  m_position = position;
}

void IPhysics::RigidBody::SetOrientation(const Quaternion& quaternion) {
  m_orientation = quaternion;
}

void IPhysics::RigidBody::SetMass(real mass) { m_inverseMass = 1 / mass; }

void IPhysics::RigidBody::SetIsAwake(bool isAwake) { m_isAwake = isAwake; }

void IPhysics::RigidBody::SetInverseMass(real inverseMass) {
  m_inverseMass = inverseMass;
}

void IPhysics::RigidBody::SetLinearDamping(real linearDamping) {
  m_linearDamping = linearDamping;
}

void IPhysics::RigidBody::SetAngularDamping(real angularDamping) {
  m_angularDamping = angularDamping;
}

void IPhysics::RigidBody::SetInverseInertiaTensor(
    const Matrix3& inertiaTensor) {
  m_inverseInertiaTensor.SetInverse(inertiaTensor);
}

void IPhysics::RigidBody::SetMaxDistanceFromCentre(real distance) {
  m_maxDistanceFromCentre = distance;
}

void IPhysics::RigidBody::AddVelocity(const Vector3& velocity) {
  m_velocity += velocity;
}

void IPhysics::RigidBody::AddRotation(const Vector3& rotation) {
  m_rotation += rotation;
}

// Queries
IPhysics::Vector3 IPhysics::RigidBody::GetPointInLocalSpace(
    const Vector3& point) const {
  return m_transformMatrix.TransformInverse(point);
}

IPhysics::Vector3 IPhysics::RigidBody::GetPointInWorldSpace(
    const Vector3& point) const {
  return m_transformMatrix.Transform(point);
}

IPhysics::real IPhysics::RigidBody::GetMass() const {
  return 1 / m_inverseMass;
}

bool IPhysics::RigidBody::GetIsAwake() const { return m_isAwake; }

IPhysics::real IPhysics::RigidBody::GetInverseMass() const {
  return m_inverseMass;
}

const IPhysics::Matrix3& IPhysics::RigidBody::GetInverseInertiaTensorWorld()
    const {
  return m_inverseInertiaTensorWorld;
}

const IPhysics::Vector3& IPhysics::RigidBody::GetPosition() const {
  return m_position;
}

const IPhysics::Quaternion& IPhysics::RigidBody::GetOrientation() const {
  return m_orientation;
}

const IPhysics::Vector3& IPhysics::RigidBody::GetRotation() const {
  return m_rotation;
}

const IPhysics::Vector3& IPhysics::RigidBody::GetVelocity() const {
  return m_velocity;
}

const IPhysics::Vector3& IPhysics::RigidBody::GetAcceleration() const {
  return m_lastFrameAcceleration;
}

const IPhysics::Vector3& IPhysics::RigidBody::GetForce() const {
  return {m_lastFrameAcceleration * (1 / m_inverseMass)};
}

const IPhysics::Matrix4& IPhysics::RigidBody::GetTransformMatrix() const {
  return m_transformMatrix;
}

const IPhysics::Vector3& IPhysics::RigidBody::GetLastFrameAcceleration() const {
  return m_lastFrameAcceleration;
}

IPhysics::real IPhysics::RigidBody::GetMaxDistanceFromCentre() const {
  return m_maxDistanceFromCentre;
}

bool IPhysics::RigidBody::HasFiniteMass() const {
  return m_inverseMass >= 0.0f;
}

void IPhysics::RigidBody::CalculateTransformMatrix(
    Matrix4& transformMatrix, const Vector3& position,
    const Quaternion& orientation) {
  transformMatrix.data[0] =
      1 - 2 * orientation.j * orientation.j - 2 * orientation.k * orientation.k;
  transformMatrix.data[1] =
      2 * orientation.i * orientation.j - 2 * orientation.r * orientation.k;
  transformMatrix.data[2] =
      2 * orientation.i * orientation.k + 2 * orientation.r * orientation.j;
  transformMatrix.data[3] = position.x;
  transformMatrix.data[4] =
      2 * orientation.i * orientation.j + 2 * orientation.r * orientation.k;
  transformMatrix.data[5] =
      1 - 2 * orientation.i * orientation.i - 2 * orientation.k * orientation.k;
  transformMatrix.data[6] =
      2 * orientation.j * orientation.k - 2 * orientation.r * orientation.i;
  transformMatrix.data[7] = position.y;
  transformMatrix.data[8] =
      2 * orientation.i * orientation.k - 2 * orientation.r * orientation.j;
  transformMatrix.data[9] =
      2 * orientation.j * orientation.k + 2 * orientation.r * orientation.i;
  transformMatrix.data[10] =
      1 - 2 * orientation.i * orientation.i - 2 * orientation.j * orientation.j;
  transformMatrix.data[11] = position.z;
}

void IPhysics::RigidBody::CalculateTransformInertiaTensor(
    Matrix3& iitWorld, const Quaternion& quaternion, const Matrix3& iitBody,
    const Matrix4& rotmat) {
  real t4 = rotmat.data[0] * iitBody.data[0] +
            rotmat.data[1] * iitBody.data[3] + rotmat.data[2] * iitBody.data[6];
  real t9 = rotmat.data[0] * iitBody.data[1] +
            rotmat.data[1] * iitBody.data[4] + rotmat.data[2] * iitBody.data[7];
  real t14 = rotmat.data[0] * iitBody.data[2] +
             rotmat.data[1] * iitBody.data[5] +
             rotmat.data[2] * iitBody.data[8];
  real t28 = rotmat.data[4] * iitBody.data[0] +
             rotmat.data[5] * iitBody.data[3] +
             rotmat.data[6] * iitBody.data[6];
  real t33 = rotmat.data[4] * iitBody.data[1] +
             rotmat.data[5] * iitBody.data[4] +
             rotmat.data[6] * iitBody.data[7];
  real t38 = rotmat.data[4] * iitBody.data[2] +
             rotmat.data[5] * iitBody.data[5] +
             rotmat.data[6] * iitBody.data[8];
  real t52 = rotmat.data[8] * iitBody.data[0] +
             rotmat.data[9] * iitBody.data[3] +
             rotmat.data[10] * iitBody.data[6];
  real t57 = rotmat.data[8] * iitBody.data[1] +
             rotmat.data[9] * iitBody.data[4] +
             rotmat.data[10] * iitBody.data[7];
  real t62 = rotmat.data[8] * iitBody.data[2] +
             rotmat.data[9] * iitBody.data[5] +
             rotmat.data[10] * iitBody.data[8];
  iitWorld.data[0] =
      t4 * rotmat.data[0] + t9 * rotmat.data[1] + t14 * rotmat.data[2];
  iitWorld.data[1] =
      t4 * rotmat.data[4] + t9 * rotmat.data[5] + t14 * rotmat.data[6];
  iitWorld.data[2] =
      t4 * rotmat.data[8] + t9 * rotmat.data[9] + t14 * rotmat.data[10];
  iitWorld.data[3] =
      t28 * rotmat.data[0] + t33 * rotmat.data[1] + t38 * rotmat.data[2];
  iitWorld.data[4] =
      t28 * rotmat.data[4] + t33 * rotmat.data[5] + t38 * rotmat.data[6];
  iitWorld.data[5] =
      t28 * rotmat.data[8] + t33 * rotmat.data[9] + t38 * rotmat.data[10];
  iitWorld.data[6] =
      t52 * rotmat.data[0] + t57 * rotmat.data[1] + t62 * rotmat.data[2];
  iitWorld.data[7] =
      t52 * rotmat.data[4] + t57 * rotmat.data[5] + t62 * rotmat.data[6];
  iitWorld.data[8] =
      t52 * rotmat.data[8] + t57 * rotmat.data[9] + t62 * rotmat.data[10];
}
