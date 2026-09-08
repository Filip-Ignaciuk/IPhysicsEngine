#include "rayguihelper.hpp"
#include <string>

IApp::CharBufferResultStore* IApp::CharBufferToReal(char buffer[64]){
    auto* charBufferResultStore = new CharBufferResultStore();
    charBufferResultStore->isValid = true;
    std::string stringForm;
    // Check if is digit
    for (size_t i = 0; i < 64; i++)
    {
        if(buffer[i] == '\0'){
            break;
        }


        if(buffer[i] !=  '.' && buffer[i] != '\0' && !std::isdigit(buffer[i])){
            charBufferResultStore->isValid = false;
            return charBufferResultStore;
        }

        stringForm = stringForm + buffer[i];

    }
    if(stringForm.size() == 0){
        charBufferResultStore->result = 0.0;
        charBufferResultStore->isValid = false;
    }
    else{
        charBufferResultStore->result = std::stod(stringForm);
    }
    return charBufferResultStore;
}