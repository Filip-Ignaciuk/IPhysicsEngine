#ifndef IPHYSICS_LANGUAGEMANAGER_HPP
#define IPHYSICS_LANGUAGEMANAGER_HPP
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "json.hpp"

namespace IApp {
class LanguageManager {
 private:
  static const std::string languagesDirectory;
  static nlohmann::json currentLanguage;
  static std::unordered_map<std::string, std::string> extensions;

 public:
  static void Initialise();
  static void LoadLanguage(std::string& filename);
  static std::string GetText(const std::string& key);
  static std::vector<std::string> GetLanguageStrings();
};
}  // namespace IApp

#endif