#ifndef IPHYSICS_INFORMATION_HPP
#define IPHYSICS_INFORMATION_HPP
#include <string>

#include "component.hpp"

namespace IPhysics {
class Information : public Component {
 private:
  std::string m_name;

 public:
  Information();

  std::string GetName();
  void SetName(std::string& name);
};
}  // namespace IPhysics

#endif