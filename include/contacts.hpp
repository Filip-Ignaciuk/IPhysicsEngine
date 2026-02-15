#pragma once
#include "core.hpp"

namespace IPhysicsEngine{
    class Contact{
        private:
        Vector3 m_contactPoint;
        Vector3 m_contactNormal;
        real m_penetration;
    };
    
}