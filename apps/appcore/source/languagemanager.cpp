#include "languagemanager.hpp"


#if defined(__APPLE__) && !defined(__EMSCRIPTEN__)
// When launching an exe on apple it launches the exe within the user home directory.
const std::string IApp::LanguageManager::languagesDirectory = "Projects/IPhysicsEngine/build/resources/languages/";
#else
const std::string IApp::LanguageManager::languagesDirectory = "resources/languages/";
#endif
nlohmann::json IApp::LanguageManager::currentLanguage;
std::unordered_map<std::string, std::string> IApp::LanguageManager::extensions;

void IApp::LanguageManager::Initialise(){
    extensions.emplace("English (UK)" , "en-GB.json");
    extensions.emplace("English (US)" , "en-US.json");
    extensions.emplace("Español" , "es.json");
    extensions.emplace("Français" , "fr.json");
    extensions.emplace("Nederlands" , "nl.json");
    //extensions.emplace("Polski" , "pl.json"); // Not supported for now :(
    extensions.emplace("Svenska" , "sv.json");
}

void IApp::LanguageManager::LoadLanguage(std::string& language){
    std::ifstream file(languagesDirectory + extensions.at(language));
    if(file){
        currentLanguage.clear();
        currentLanguage = nlohmann::json::parse(file);
    }
}

std::string IApp::LanguageManager::GetText(const std::string& key){
    return currentLanguage[key].get<std::string>();
}

std::vector<std::string> IApp::LanguageManager::GetLanguageStrings(){
    std::vector<std::string> languages;
    for(auto& language : extensions){
        languages.emplace_back(language.first);
    }
    return languages;
}
