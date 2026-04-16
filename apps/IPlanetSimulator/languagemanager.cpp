#include "languagemanager.hpp"
#include <fstream>
#include "json.hpp"

nlohmann::json IPhysics::LanguageManager::language;

void IPhysics::LanguageManager::LoadLanguage(std::string& _filename){
    std::ifstream file(_filename);
    language = nlohmann::json::parse(file);
}

std::string IPhysics::LanguageManager::GetText(const std::string& _key){
    return language[_key].get<std::string>();
}