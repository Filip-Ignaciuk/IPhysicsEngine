#include "rayguihelper.hpp"
#include <string>

IApp::CharBufferResultStore* IApp::CharBufferToReal(char _buffer[64]){
    auto* charBufferResultStore = new CharBufferResultStore();
    charBufferResultStore->isValid = true;
    std::string stringForm;
    // Check if is digit
    for (size_t i = 0; i < 64; i++)
    {
        if(_buffer[i] == '\0'){
            break;
        }


        if(_buffer[i] !=  '.' && _buffer[i] != '\0' && !std::isdigit(_buffer[i])){
            charBufferResultStore->isValid = false;
            return charBufferResultStore;
        }

        stringForm = stringForm + _buffer[i];

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