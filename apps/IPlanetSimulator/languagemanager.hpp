#pragma once
#include <string>
#include <map>

#include "json.hpp"

namespace IPhysics{
    class LanguageManager{
        private:
        static nlohmann::json language;
        public:
        static void LoadLanguage(std::string& _filename);
        static std::string GetText(const std::string& _key);
    };
}