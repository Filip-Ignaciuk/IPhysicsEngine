#ifndef IPHYSICS_RIGIDBODY_HPP
#define IPHYSICS_RIGIDBODY_HPP
#include "component.hpp"
#include "core.hpp"
#include "precision.hpp"
namespace IPhysics {
class RigidBody : public Component {
 public:
  // Constructors
  RigidBody();
  RigidBody(const Vector3& originalPosition,
            const Quaternion& originalOrientation, const real& inverseMass,
            const real& linearDamping, const real& angularDamping,
            const Matrix3& inverseInertiaTensor);

  // Mutators
  void Integrate(real duration);
  void AddForce(const Vector3& vector);
  void AddForceAtPoint(const Vector3& vector, const Vector3& point);
  void AddForceAtBodyPoint(const Vector3& vector, const Vector3& point);
  void ClearAccumulators();
  void CalculateDerivedData();

  void SetPosition(const Vector3& position);
  void SetOrientation(const Quaternion& quaternion);
  void SetMass(real mass);
  void SetIsAwake(bool isAwake);
  void SetInverseMass(real inverseMass);
  void SetLinearDamping(real linearDamping);
  void SetAngularDamping(real angularDamping);
  void SetInverseInertiaTensor(const Matrix3& inertiaTensor);
  void SetMaxDistanceFromCentre(real distance);

  void AddVelocity(const Vector3& velocity);
  void AddRotation(const Vector3& rotation);

  // Queries
  [[nodiscard]] Vector3 GetPointInLocalSpace(const Vector3& point) const;
  [[nodiscard]] Vector3 GetPointInWorldSpace(const Vector3& point) const;
  [[nodiscard]] real GetMass() const;
  [[nodiscard]] bool GetIsAwake() const;
  [[nodiscard]] real GetInverseMass() const;
  [[nodiscard]] const Matrix3& GetInverseInertiaTensorWorld() const;
  [[nodiscard]] const Vector3& GetPosition() const;
  [[nodiscard]] const Quaternion& GetOrientation() const;
  [[nodiscard]] const Vector3& GetRotation() const;
  [[nodiscard]] const Vector3& GetVelocity() const;
  [[nodiscard]] const Vector3& GetAcceleration() const;
  [[nodiscard]] const Vector3& GetForceAccumulated() const;
  [[nodiscard]] const Matrix4& GetTransformMatrix() const;
  [[nodiscard]] const Vector3& GetLastFrameAcceleration() const;
  [[nodiscard]] real GetMaxDistanceFromCentre() const;
  [[nodiscard]] bool HasFiniteMass() const;

 protected:
  // Properties
  Vector3 m_position;
  Quaternion m_orientation;
  real m_inverseMass{};
  Matrix3 m_inverseInertiaTensor;

  // Linear
  real m_linearDamping{};
  Vector3 m_velocity;
  Vector3 m_acceleration;
  Vector3 m_forceAccumulated;

  // Angular
  real m_angularDamping{};
  Vector3 m_rotation;
  Vector3 m_torqueAccumulated;

  Matrix3 m_inverseInertiaTensorWorld;
  real m_maxDistanceFromCentre{};
  bool m_isAwake{};
  Matrix4 m_transformMatrix;
  Vector3 m_lastFrameAcceleration;

 private:
  static void CalculateTransformMatrix(Matrix4& transformMatrix,
                                       const Vector3& position,
                                       const Quaternion& orientation);
                                       
  static void CalculateTransformInertiaTensor(Matrix3& iitWorld,
                                              const Quaternion& quaternion,
                                              const Matrix3& iitBody,
                                              const Matrix4& rotationMatrix);
};
}  // namespace IPhysics

#endif
