#ifndef IPHYSICS_RAYGUIHELPER_HPP
#define IPHYSICS_RAYGUIHELPER_HPP
#include "precision.hpp"



namespace IApp {
  constexpr int textIntegerBufferLimit = 20;

  // Class to represent text input from a user.
struct CharBufferResultStore {
  // Represents if the text is a valid
  bool isValid;
  IPhysics::real result;
};

CharBufferResultStore* CharBufferToReal(char buffer[textIntegerBufferLimit]);
}  // namespace IApp

#endif
