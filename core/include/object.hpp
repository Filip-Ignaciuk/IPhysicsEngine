#ifndef IPHYSICS_OBJECT_HPP
#define IPHYSICS_OBJECT_HPP
#include <iterator>
#include <memory>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

#include "components/component.hpp"

namespace IPhysics {
class Object {
 private:
  typedef std::unordered_map<std::type_index, std::unique_ptr<Component>> Map;
  Map components;

 public:
  Object();

  template <typename T>
  T* AddComponent() {
    std::unique_ptr<T> componentPointer = std::make_unique<T>();
    T* pointer = componentPointer.get();
    components.emplace(std::type_index(typeid(T)),
                       std::unique_ptr<Component>(std::move(componentPointer)));
    return pointer;
  }

  template <typename T>
  T* GetComponent() {
    Map::iterator componentIterator =
        components.find(std::type_index(typeid(T)));

    if (componentIterator != components.end()) {
      return dynamic_cast<T*>(componentIterator->second.get());
    }

    return nullptr;
  }

  template <typename T>
  void RemoveComponent() {
    components.erase(std::type_index(typeid(T)));
  }
};

}  // namespace IPhysics

#endif