#ifndef IPHYSICS_RAYGUIHELPER_HPP
#define IPHYSICS_RAYGUIHELPER_HPP
#include "precision.hpp"

namespace IApp {
struct CharBufferResultStore {
  bool isValid;
  IPhysics::real result;
};

CharBufferResultStore* CharBufferToReal(char buffer[64]);
}  // namespace IApp

#endif
