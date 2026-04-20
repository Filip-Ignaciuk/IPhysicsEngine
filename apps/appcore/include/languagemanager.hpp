#pragma once
#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <vector>
#include "json.hpp"

namespace IApp{
    class LanguageManager{
        private:
        static const std::string languagesDirectory;
        static nlohmann::json currentLanguage;
        static std::unordered_map<std::string, std::string> extensions;
        public:
        static void Initialise();
        static void LoadLanguage(std::string& _filename);
        static std::string GetText(const std::string& _key);
        static std::vector<std::string> GetLanguageStrings();
    };
}