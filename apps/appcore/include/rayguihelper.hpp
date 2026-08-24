#pragma once
#include "precision.hpp"


namespace IApp {
    struct CharBufferResultStore{
        bool isValid;
        IPhysics::real result;
    };

    CharBufferResultStore* CharBufferToReal(char _buffer[64]);
}


