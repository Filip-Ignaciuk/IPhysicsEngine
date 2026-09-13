#ifndef IPHYSICS_WORLD_HPP
#define IPHYSICS_WORLD_HPP

#include <algorithm>
#include <vector>

#include "collidebroad.hpp"
#include "collidenarrow.hpp"
#include "contacts.hpp"
#include "forcegenerator.hpp"
#include "object.hpp"

namespace IPhysics {
class World {
 public:
  // Mutators
  void StartFrame();
  void RunPhysics(real timestep);

  void AddObject(Object* object);
  void RemoveObject(Object* object);
  void RemoveLastObject();
  void RemoveAllObjects();

  void AddForceRegistration(
      Object* object, const std::shared_ptr<ForceGenerator>& forceGenerator);
  void RemoveForceRegistration(Object* object);

  void SetPhysicsState(bool state);

  // Queries
  [[nodiscard]] const std::vector<Object*>& GetObjects();
  [[nodiscard]] Object* GetLastObject();
  [[nodiscard]] const int GetNumberOfObjects() const;
  [[nodiscard]] const ForceRegistration& GetForceRegistration(
      Object* object) const;
  [[nodiscard]] const ForceRegistry& GetForceRegistry() const;
  [[nodiscard]] bool GetPhysicsState() const;

 protected:
  static constexpr unsigned MAX_CONTACTS = 256;

  std::vector<Object*> m_objects;
  bool m_physicsState = true;

  BoundingVolumeHierarchyNode<BoundingSphere>* m_root = nullptr;
  ContactResolver m_contactResolver{};
  ForceRegistry m_registry;
};
}  // namespace IPhysics

#endif