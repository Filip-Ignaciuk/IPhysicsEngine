#include "languagemanager.hpp"
#include <fstream>
#include "json.hpp"

nlohmann::json IPhysicsEngine::LanguageManager::language;

void IPhysicsEngine::LanguageManager::LoadLanguage(std::string& _filename){
    std::ifstream file(_filename);
    language = nlohmann::json::parse(file);
}

std::string IPhysicsEngine::LanguageManager::GetText(const std::string& _key){
    return language[_key].get<std::string>();
}