#pragma once
#include "component.hpp"
#include <string>

namespace IPhysics{
    class Information : public Component{
        private:
        std::string m_name;

        public:
        Information();

        std::string GetName();
        void SetName(std::string& _name);

    };
}