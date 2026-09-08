#ifndef IPHYSICS_COLLIDEBROAD_HPP
#define IPHYSICS_COLLIDEBROAD_HPP
#include "components/rigidbody.hpp"
#include "core.hpp"
#include "object.hpp"
namespace IPhysics {
struct PotentialContact {
  Object* object[2];
};

template <class BoundingVolumeClass>
class BoundingVolumeHierarchyNode {
 public:
  BoundingVolumeHierarchyNode* children[2];

  BoundingVolumeClass volume;

  Object* object;

  BoundingVolumeHierarchyNode* parent;

  BoundingVolumeHierarchyNode(BoundingVolumeHierarchyNode* parent,
                              const BoundingVolumeClass volume, Object* object)
      : parent(parent), volume(volume), object(object) {
    children[0] = children[1] = nullptr;
  }

  bool IsLeaf() const { return object != nullptr; }

  unsigned GetPotentialContacts(PotentialContact* contacts,
                                unsigned limit) const {
    if (IsLeaf() || limit == 0) {
      return 0;
    }
    return children[0]->GetPotentialContactsWith(children[1], contacts, limit);
  }

  void Insert(Object* newObject, const BoundingVolumeClass newVolume) {
    if (IsLeaf()) {
      children[0] = new BoundingVolumeHierarchyNode<BoundingVolumeClass>(
          this, volume, object);

      children[1] = new BoundingVolumeHierarchyNode<BoundingVolumeClass>(
          this, newVolume, newObject);

      this->object = nullptr;

      RecalculateBoundingVolume();
    } else {
      if (children[0]->volume.GetGrowth(newVolume) <
          children[1]->volume.GetGrowth(newVolume)) {
        children[0]->Insert(newObject, newVolume);
      } else {
        children[1]->Insert(newObject, newVolume);
      }
    }
  }

  ~BoundingVolumeHierarchyNode() {
    // If we don't have a parent, then we ignore the sibling.
    if (parent) {
      BoundingVolumeHierarchyNode<BoundingVolumeClass>* sibling;
      // Find our sibling
      if (parent->children[0] == this) {
        sibling = parent->children[1];
      } else {
        sibling = parent->children[0];
      }

      parent->volume = sibling->volume;
      parent->object = sibling->object;
      parent->children[0] = sibling->children[0];
      parent->children[1] = sibling->children[1];

      // Delete the sibling
      sibling->parent = nullptr;
      sibling->object = nullptr;
      sibling->children[0] = nullptr;
      sibling->children[1] = nullptr;
      delete sibling;

      // Recalculate the parent's bounding volume.
      parent->RecalculateBoundingVolume();
    }

    if (children[0]) {
      children[0]->parent = nullptr;
      delete children[0];
    }
    if (children[1]) {
      children[1]->parent = nullptr;
      delete children[1];
    }
  }

 protected:
  bool Overlaps(
      const BoundingVolumeHierarchyNode<BoundingVolumeClass>* other) const {
    return volume.Overlaps(&other->volume);
  }

  unsigned GetPotentialContactsWith(
      const BoundingVolumeHierarchyNode<BoundingVolumeClass>* other,
      PotentialContact* contacts, unsigned limit) const {
    if (!Overlaps(other) || limit == 0) {
      return 0;
    }

    if (IsLeaf() && other->IsLeaf()) {
      contacts->object[0] = object;
      contacts->object[1] = other->object;
      return 1;
    }

    if (other->IsLeaf() ||
        (!IsLeaf() && volume.GetSize() >= other->volume.GetSize())) {
      unsigned count =
          children[0]->GetPotentialContactsWith(other, contacts, limit);

      if (limit > count) {
        return count + children[1]->GetPotentialContactsWith(
                           other, contacts + count, limit - count);
      } else {
        return count;
      }
    } else {
      unsigned count =
          GetPotentialContactsWith(other->children[0], contacts, limit);

      if (limit > count) {
        return count + GetPotentialContactsWith(
                           other->children[1], contacts + count, limit - count);
      } else
        return count;
    }
  }

  void RecalculateBoundingVolume(bool recurse = true) {
    if (IsLeaf()) {
      return;
    }
    volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);
    if (parent) {
      parent->RecalculateBoundingVolume(true);
    }
  }
};
}  // namespace IPhysics

namespace IPhysics {
struct BoundingSphere {
  Vector3 m_centre;
  real m_radius;
  BoundingSphere(const Vector3& centre, real radius);
  BoundingSphere(const BoundingSphere& one, const BoundingSphere& two);
  bool Overlaps(const BoundingSphere* other) const;
  real GetGrowth(const BoundingSphere& other) const;
  real GetSize() const;
};
}  // namespace IPhysics

#endif