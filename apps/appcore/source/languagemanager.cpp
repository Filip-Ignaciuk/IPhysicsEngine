#include "languagemanager.hpp"
#include <fstream>
#include "json.hpp"

nlohmann::json IApp::LanguageManager::language;

void IApp::LanguageManager::LoadLanguage(std::string& _filename){
    std::ifstream file(_filename);
    language = nlohmann::json::parse(file);
}

std::string IApp::LanguageManager::GetText(const std::string& _key){
    return language[_key].get<std::string>();
}